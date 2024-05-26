if ! cmp -s patch/autoware/autoware_map/main-map/merging.osm patch/autoware/autoware_map/main-map/lanelet2_map.osm; then
    cp patch/autoware/autoware_map/main-map/merging.osm patch/autoware/autoware_map/main-map/lanelet2_map.osm
    echo 'finished' > patch/autoware/config/v1/dashboard_status.txt
    echo 'finished' > patch/autoware/config/v2/dashboard_status.txt
    echo 'finished' > patch/autoware/config/v3/dashboard_status.txt
    echo 'finished' > patch/autoware/config/v4/dashboard_status.txt
fi
