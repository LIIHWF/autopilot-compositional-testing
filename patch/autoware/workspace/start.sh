#!/bin/bash
cd $HOME/workspace
source /autoware/install/setup.bash
./scripts/guard_dashboard.sh &
./scripts/client.sh
