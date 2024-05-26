rocker --nvidia --x11 --user \
    --hostname $1 \
    --volume $PWD/patch/autoware/autoware_map:$HOME/autoware_map \
    --volume $PWD/patch/autoware/autoware_data:$HOME/autoware_data \
    --volume $PWD/patch/autoware/workspace:$HOME/workspace \
    --volume $PWD/patch/autoware/config/$1:$HOME/config \
    --env ROS_DOMAIN_ID=$(cat $HOME/ADS/autoware/config/$1/ROS_DOMAIN_ID) \
    --name "autoware_docker_$1" -- llccc/autoware-v1-determinism:latest \
    $HOME/workspace/start.sh
