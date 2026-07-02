#include "behaviortree_ros2/tree_execution_server.hpp"

#include <cctype>
#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace robotino_behavior_tree
{
namespace
{

std::string trim(const std::string & input)
{
  std::size_t first = 0;
  while(first < input.size() && std::isspace(static_cast<unsigned char>(input[first]))) {
    ++first;
  }

  std::size_t last = input.size();
  while(last > first && std::isspace(static_cast<unsigned char>(input[last - 1]))) {
    --last;
  }

  return input.substr(first, last - first);
}

class PayloadParser
{
public:
  explicit PayloadParser(const std::string & input)
  : input_(input)
  {}

  std::map<std::string, std::string> parse()
  {
    std::map<std::string, std::string> values;
    skipWhitespace();
    expect('{');
    skipWhitespace();

    if(consume('}')) {
      skipWhitespace();
      expectEnd();
      return values;
    }

    while(true) {
      skipWhitespace();
      const auto key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();
      values[key] = parseValue();
      skipWhitespace();

      if(consume('}')) {
        break;
      }
      expect(',');
    }

    skipWhitespace();
    expectEnd();
    return values;
  }

private:
  void skipWhitespace()
  {
    while(pos_ < input_.size() && std::isspace(static_cast<unsigned char>(input_[pos_]))) {
      ++pos_;
    }
  }

  bool consume(char expected)
  {
    if(pos_ < input_.size() && input_[pos_] == expected) {
      ++pos_;
      return true;
    }
    return false;
  }

  void expect(char expected)
  {
    if(!consume(expected)) {
      throw std::runtime_error(std::string("expected '") + expected + "'");
    }
  }

  void expectEnd()
  {
    if(pos_ != input_.size()) {
      throw std::runtime_error("unexpected characters after payload object");
    }
  }

  std::string parseString()
  {
    expect('"');
    std::string output;

    while(pos_ < input_.size()) {
      const char current = input_[pos_++];
      if(current == '"') {
        return output;
      }
      if(current != '\\') {
        output.push_back(current);
        continue;
      }
      if(pos_ >= input_.size()) {
        throw std::runtime_error("unterminated escape sequence");
      }

      const char escaped = input_[pos_++];
      switch(escaped) {
        case '"':
        case '\\':
        case '/':
          output.push_back(escaped);
          break;
        case 'b':
          output.push_back('\b');
          break;
        case 'f':
          output.push_back('\f');
          break;
        case 'n':
          output.push_back('\n');
          break;
        case 'r':
          output.push_back('\r');
          break;
        case 't':
          output.push_back('\t');
          break;
        default:
          throw std::runtime_error("unsupported escape sequence in payload string");
      }
    }

    throw std::runtime_error("unterminated string in payload");
  }

  std::string parseValue()
  {
    if(pos_ < input_.size() && input_[pos_] == '"') {
      return parseString();
    }

    const auto first = pos_;
    while(pos_ < input_.size() && input_[pos_] != ',' && input_[pos_] != '}') {
      ++pos_;
    }
    return trim(input_.substr(first, pos_ - first));
  }

  const std::string & input_;
  std::size_t pos_ = 0;
};

std::map<std::string, std::string> parsePayload(const std::string & payload)
{
  const auto stripped_payload = trim(payload);
  if(stripped_payload.empty()) {
    return {};
  }
  return PayloadParser(stripped_payload).parse();
}

}  // namespace

class RobotinoTreeServer : public BT::TreeExecutionServer
{
public:
  explicit RobotinoTreeServer(const rclcpp::NodeOptions & options)
  : BT::TreeExecutionServer(options)
  {
    try {
      executeRegistration();
    } catch(const std::exception & error) {
      RCLCPP_FATAL(node()->get_logger(),
                   "Failed to register behavior tree plugins/trees: %s", error.what());
      throw;
    }
  }

protected:
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override
  {
    RCLCPP_INFO(node()->get_logger(), "ExecuteTree goal received: tree=%s payload=%s",
                tree_name.c_str(), payload.c_str());
    try {
      parsePayload(payload);
    } catch(const std::exception & error) {
      RCLCPP_ERROR(node()->get_logger(), "Invalid ExecuteTree payload: %s", error.what());
      return false;
    }
    return true;
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    auto blackboard = tree.rootBlackboard();
    const auto values = parsePayload(goalPayload());
    const auto valueOrEmpty = [&values](const std::string & key) -> std::string {
      const auto value_it = values.find(key);
      return value_it == values.end() ? std::string{} : value_it->second;
    };

    RCLCPP_INFO(node()->get_logger(),
                "ExecuteTree blackboard: machine_input_tf=%s reference_frame=%s "
                "object_type=%s lego_color=%s action=%s object_prompt=%s",
                valueOrEmpty("machine_input_tf").c_str(),
                valueOrEmpty("reference_frame").c_str(),
                valueOrEmpty("object_type").c_str(),
                valueOrEmpty("lego_color").c_str(),
                valueOrEmpty("action").c_str(),
                valueOrEmpty("object_prompt").c_str());

    setIfPresent(blackboard, values, "object_prompt");
    setIfPresent(blackboard, values, "machine_input_tf");
    setIfPresent(blackboard, values, "reference_frame");
    setIfPresent(blackboard, values, "object_type");
    setIfPresent(blackboard, values, "lego_color");
    setIfPresent(blackboard, values, "action");
  }

private:
  static void setIfPresent(
    const BT::Blackboard::Ptr & blackboard,
    const std::map<std::string, std::string> & values,
    const std::string & key)
  {
    const auto value_it = values.find(key);
    if(value_it != values.end() && !value_it->second.empty()) {
      blackboard->set(key, value_it->second);
    }
  }
};

}  // namespace robotino_behavior_tree

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    auto action_server = std::make_shared<robotino_behavior_tree::RobotinoTreeServer>(options);

    auto node = action_server->node();
    std::string ns = node->get_namespace();
    // Strip leading slash if present
    if (!ns.empty() && ns[0] == '/') {
      ns = ns.substr(1);
    }
    action_server->globalBlackboard()->set("namespace", ns);

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                  std::chrono::milliseconds(250));
    exec.add_node(action_server->node());
    exec.spin();
    exec.remove_node(action_server->node());
  } catch(const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("robotino_tree_server"),
                 "Unhandled exception in robotino_tree_server: %s", error.what());
    rclcpp::shutdown();
    return 1;
  } catch(...) {
    RCLCPP_FATAL(rclcpp::get_logger("robotino_tree_server"),
                 "Unhandled non-standard exception in robotino_tree_server");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
