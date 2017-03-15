#!//bin/bash

probability_options="0.25 0.5 0.75"
eval_time_options="10000 100000 500000"

for probability in $probability_options; do
  for eval_time in $eval_time_options; do
    rosrun esp_planner 2dnav_experiments /usr0/home/venkatrn/indigo_workspace/src/esp_planner/willow_map.jpg $probability $eval_time
  done
done
