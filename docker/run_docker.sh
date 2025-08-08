#!/bin/bash
IMAGE_NAME="reg.supremind.info/product/data/ai_infra/lerobot-cpu-ros2:20250721113600 "

docker run -it -v $(pwd):/bt_ros2_parent $IMAGE_NAME /bin/bash