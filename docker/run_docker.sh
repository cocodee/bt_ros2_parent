#!/bin/bash
IMAGE_NAME="misumi_base:ros2"

docker run -it -v $(pwd):/bt_ros2_parent $IMAGE_NAME /bin/bash