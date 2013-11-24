#!/bin/sh
interval=0.5
while :
do
    rosservice call /joint_state_buffer_server/update "/atlas/joint_states_compressed"
    sleep $interval

    rosservice call /joint_state_buffer_server/update "/multisense_sl/joint_states_compressed"
    sleep $interval
done
