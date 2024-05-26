ps -u | grep "bridge_client" | grep -v "grep" | awk '{print $2;}' | xargs kill -9
