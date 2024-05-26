source /autoware/install/setup.bash

sudo chmod 777 /autoware/install/dummy_perception_publisher/share/dummy_perception_publisher/launch/dummy_perception_publisher.launch.xml
echo '<?xml version="1.0"?>
<launch>
  <!-- dummy detection and sensor data -->
  <arg name="visible_range" default="300.0"/>
  <arg name="real" default="true"/>
  <arg name="use_object_recognition" default="true"/>
  <arg name="use_base_link_z" default="true"/>

  <group>
    <push-ros-namespace namespace="simulation"/>
    <group if="$(var real)">
      <node pkg="dummy_perception_publisher" exec="dummy_perception_publisher_node" name="dummy_perception_publisher" output="screen">
        <remap from="output/dynamic_object" to="/perception/object_recognition/detection/labeled_clusters"/>
        <remap from="output/objects_pose" to="debug/object_pose"/>
        <remap from="output/points_raw" to="/perception/obstacle_segmentation/pointcloud"/>
        <remap from="input/object" to="dummy_perception_publisher/object_info"/>
        <remap from="input/reset" to="input/reset"/>
        <param name="visible_range" value="$(var visible_range)"/>
        <param name="detection_successful_rate" value="0.999"/>
        <param name="enable_ray_tracing" value="false"/>
        <param name="use_object_recognition" value="$(var use_object_recognition)"/>
        <param name="object_centric_pointcloud" value="false"/>
        <param name="use_base_link_z" value="$(var use_base_link_z)"/>
        <param name="publish_ground_truth" value="true"/>
        <remap from="output/debug/ground_truth_objects" to="debug/ground_truth_objects"/>
      </node>
      <include file="$(find-pkg-share shape_estimation)/launch/shape_estimation.launch.xml">
        <arg name="input/objects" value="/perception/object_recognition/detection/labeled_clusters"/>
        <arg name="output/objects" value="/perception/object_recognition/detection/objects_with_feature"/>
      </include>
    </group>
    <group unless="$(var real)">
      <node pkg="dummy_perception_publisher" exec="dummy_perception_publisher_node" name="dummy_perception_publisher" output="screen">
        <remap from="output/dynamic_object" to="/perception/object_recognition/detection/objects_with_feature"/>
        <remap from="output/objects_pose" to="debug/object_pose"/>
        <remap from="output/points_raw" to="/perception/obstacle_segmentation/pointcloud"/>
        <remap from="input/object" to="dummy_perception_publisher/object_info"/>
        <remap from="input/reset" to="input/reset"/>
        <param name="visible_range" value="$(var visible_range)"/>
        <param name="detection_successful_rate" value="1.0"/>
        <param name="enable_ray_tracing" value="false"/>
        <param name="use_object_recognition" value="$(var use_object_recognition)"/>
        <param name="use_base_link_z" value="$(var use_base_link_z)"/>
        <param name="use_fixed_random_seed" value="true"/>
        <param name="random_seed" value="1"/>
      </node>
    </group>

    <!-- convert DynamicObjectsWithFeatureArray to DynamicObjects -->
    <include file="$(find-pkg-share detected_object_feature_remover)/launch/detected_object_feature_remover.launch.xml">
      <arg name="input" value="/perception/object_recognition/detection/objects_with_feature"/>
      <arg name="output" value="/perception/object_recognition/detection/objects"/>
    </include>
  </group>
</launch>
' > /autoware/install/dummy_perception_publisher/share/dummy_perception_publisher/launch/dummy_perception_publisher.launch.xml


source $HOME/workspace/setup.sh
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/main-map \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    use_sim_time:=true \
    perception/enable_detection_failure:=false \
    sensing/visible_range:=1000.0 \
    rviz:=true

