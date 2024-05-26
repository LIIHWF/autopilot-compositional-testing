U=$(whoami)
H=$(echo ~)

sudo chown -R $U:$U $H
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /autoware/install/setup.bash