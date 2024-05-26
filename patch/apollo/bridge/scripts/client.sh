bash /apollo/scripts/dreamview.sh
source /apollo/bridge/bashrc
pip install -r /apollo/bridge/requirements.txt
# loop to run client
while true; do
    timeout 300 python /apollo/bridge/bridge_client/client.py
    sleep 1
done
