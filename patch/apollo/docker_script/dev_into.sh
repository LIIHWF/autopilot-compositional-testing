#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
DOCKER_USER="${USER}"
DEV_CONTAINER="apollo_dev_${USER}_$1"

# Check if the container is running
if [ "$(docker ps -q -f name=$DEV_CONTAINER)" ]; then
    echo "Container $CONTAINER_NAME is already running."
else
    echo "Container $DEV_CONTAINER is not running. Starting it now..."
    docker start $DEV_CONTAINER
fi


xhost +local:root 1>/dev/null 2>&1

docker exec \
    -u "${DOCKER_USER}" \
    -e HISTFILE=/apollo/.dev_bash_hist \
    -it "${DEV_CONTAINER}" \
    /bin/bash

xhost -local:root 1>/dev/null 2>&1
