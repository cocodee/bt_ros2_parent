## Run
```shell
ros2 run my_robot_py_nodes sync_move_server
ros2 run bt_ros2 bt_executor
ros2 action send_goal --feedback /execute_command my_robot_interfaces/action/ExecuteCommand '{ "command": "start_dance" }'
```