cd /apollo
dir_name=/apollo/modules/map/data/$1
/apollo/scripts/generate_routing_topo_graph.sh --map_dir ${dir_name}
/apollo/bazel-bin/modules/map/tools/sim_map_generator --map_dir=${dir_name} --output_dir=${dir_name}
touch ${dir_name}/default_end_way_point.txt
# /apollo/apollo.sh build_opt_gpu
