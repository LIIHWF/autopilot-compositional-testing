# timeout 300 bazel-bin/src/test_data_generator/apollo_fast_search apollo-40 lane_change -ve 5
# timeout 3600 bazel-bin/src/test_data_generator/apollo_fast_search apollo-40 lane_change -ve 10
# timeout 3600 bazel-bin/src/test_data_generator/apollo_fast_search apollo-40 merging -ve 0
# timeout 3600 bazel-bin/src/test_data_generator/apollo_fast_search apollo-40 merging -ve 5
# timeout 900 bazel-bin/src/test_data_generator/apollo_fast_search apollo-40 crossing_with_yield_signs -ve 0  timeout 1800 bazel-bin/src/test_data_generator/apollo_fast_search apollo-40 crossing_with_yield_signs -ve 5

# for ve in 0 5 10; do
#     for xf in 0 10 20 30 40 50 60 70 80 90 100 110 120 130 140 150 160 170 180 190 200 210 220 230 240 250 260 270 280 290 300 310 320
#     do
#         ./script/run_single.sh apollo-40 crossing_with_traffic_lights -ve $ve -xf $xf --json > test_result/apollo-40/crossing_with_traffic_lights/ve=$ve/xf=$xf.json
#     done
# done

# timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior lane_change -ve 4
timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior lane_change -ve 8
timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior merging -ve 0
timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior merging -ve 5
timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior crossing_with_yield_signs -ve 0
timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior crossing_with_yield_signs -ve 5

timeout 900 bazel-bin/src/test_data_generator/behavior_fast_search behavior crossing_with_traffic_lights -ve 0
timeout 900 bazel-bin/src/test_data_generator/behavior_fast_search behavior crossing_with_traffic_lights -ve 5

timeout 3600 bazel-bin/src/test_data_generator/behavior_fast_search behavior lane_change -ve 4