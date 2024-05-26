export ROS_DOMAIN_ID=$(cat $HOME/config/ROS_DOMAIN_ID)
source $HOME/workspace/setup.sh

if ! pip list | grep -q loguru; then
  pip install $HOME/workspace/python_pkg/websocket-client/websocket_client-1.3.3-py3-none-any.whl
  pip install $HOME/workspace/python_pkg/loguru/loguru-0.7.2-py3-none-any.whl
  pip install $HOME/workspace/python_pkg/netifaces/netifaces-0.11.0.tar.gz
  # pip install loguru
  # pip install websocket-client==1.3.3
fi

while true; do
  python3 client/bridge_client.py
  if [ $? -eq 124 ]; then
    echo "Timeout, restarting bridge client"
    echo "finished" > $HOME/config/dashboard_status.txt
  fi
  sleep 1
done
