#!/bin/bash
killall -9 gzserver

SESSION=PLD_VISFEEDBACK

tmuxstart() {
    if [[ $(tmux has-session -t "$1") -eq 0 ]] ; then
        echo "Killing previous session with name $1"
        tmux kill-session -t  "$1"
    fi
    #rest of tmux script to create session named "sess"
    tmux new-session -d -s "$1"
}

# Do not clutter up history with commands from tmux session
unset HISTFILE
tmuxstart ${SESSION}
tmux send-keys -t ${SESSION} "tmux set-option -g remain-on-exit on"
tmux rename-window main
# Run optitrack emulator in second window!
tmux send-keys -t ${SESSION} "tmux new-window -n gcs 'roslaunch px4_command px4_multidrone_pos_estimator_pure_vision.launch uavID:=uav2'" ENTER
# Split panes then ssh to the vehicle in each pane
tmux send-keys -t ${SESSION} "roslaunch px4 single_drone_payload_vision_sitl.launch" ENTER
tmux splitw -v -p 50 "roslaunch optitrack_broadcast emulator_for_gazebo.launch"
tmux splitw -h -p 50 -t 1 "rosrun px4_command set_uav2_mode"
tmux splitw -h -p 50 -t 1 "rosrun qt_ground_station qt_ground_station"
tmux splitw -h -p 50 -t 2 "rosrun track_april_tag april_tag_opencv_emulate"
tmux splitw -h -p 50 -t 2 "roslaunch px4_command px4_multidrone_pos_controller_gazebo.launch uavID:=uav2"


gnome-terminal --tab -- tmux attach -t ${SESSION} &




## Create the windows on which each node or .launch file is going to run

