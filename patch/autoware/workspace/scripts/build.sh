#! /bin/bash
cd /autoware
colcon build --parallel-workers 24 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release