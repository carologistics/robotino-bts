# robotino_bahavior_tree

BehaviorTree.CPP plugins for the robotino mobile base and gripper stack.

## Implemented nodes

- `MotorMove`
  - Action type: `motor_move_msgs/action/MotorMove`
  - Typical server: `/{namespace}/motor_move_action`
  - Inputs: `server_name`, `frame_id`, `x`, `y`, `yaw`, optional `z`, optional `qx/qy/qz/qw`
  - Outputs: `success`, `distance_to_target`

- `MoveGripperTo`
  - Action type: `gigatino_msgs/action/Move`
  - Typical server: `/{namespace}/gigatino/move`
  - Inputs: `server_name`, `frame`, `x`, `y`, `z`
  - Outputs: `success`

## Example BT XML

```xml
<BehaviorTree ID="MainTree">
  <Sequence>
    <MotorMove
      server_name="robotino1/motor_move_action"
      frame_id="robotino1/base_link"
      x="1.0"
      y="0.0"
      yaw="1.57079632679"
      success="{motor_move_success}"
      distance_to_target="{motor_move_distance}"/>

    <MoveGripperTo
      server_name="robotino1/gigatino/move"
      frame="robotino1/base_link"
      x="0.25"
      y="0.00"
      z="0.12"
      success="{gripper_move_success}"/>
  </Sequence>
</BehaviorTree>
```

## Loading in Nav2 / BT Navigator

Add the generated shared libraries to your BT plugin library list, for example:

- `motor_move_bt_node`
- `move_gripper_to_bt_node`
