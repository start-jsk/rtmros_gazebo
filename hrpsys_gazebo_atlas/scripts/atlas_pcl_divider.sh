#!/bin/sh
filter_field_name="x"
filter_limit_negative="false"
input_frame="/pelvis"
leaf_size="0.2"
divide_num=10
filter_min="0.00"
filter_size="1.00"
output_file="`rospack find hrpsys_gazebo_atlas`/launch/atlas_pcl_divider.launch"

cat << _EOT_ > $output_file
<launch>
  <node pkg="nodelet" type="nodelet" name="pcl_divider_nodelet_manager" args="manager"
        output="screen" alaunch-prefix="xterm -e gdb --args" />
  <group ns="pcl_divider_nodelet">
_EOT_



filter_limit_max=$filter_min
i=0

while [ $i -ne $divide_num ]
do
    i=`expr $i + 1`
    filter_limit_min=$filter_limit_max
    filter_limit_max=`echo "scale=3; $filter_limit_min+$filter_size" | bc`
    topic_buffer_args="${topic_buffer_args} voxelgrid${i}/output"
    cat << _EOT_ >> $output_file
      <node pkg="nodelet" type="nodelet"
          name="voxelgrid${i}"
          args="load pcl/VoxelGrid /pcl_divider_nodelet_manager"
          output="screen" clear_params="true">
      <remap from="~input" to="/laser/full_cloud2_filtered" />
      <rosparam>
        filter_field_name: "${filter_field_name}"
        filter_limit_min: ${filter_limit_min}
        filter_limit_max: ${filter_limit_max}
        filter_limit_negative: ${filter_limit_negative}
        input_frame: "${input_frame}"
        leaf_size: ${leaf_size}
      </rosparam>
    </node>
_EOT_
done
cat << _EOT_ >> $output_file
    <node pkg="jsk_topic_tools" type="topic_buffer_server"
        name="topic_buffer_server"
        args="${topic_buffer_args}"
        output="screen"/>
_EOT_


cat << _EOT_ >> $output_file
  </group>
</launch>
_EOT_

echo "generated atlas_pcl_divider.launch"