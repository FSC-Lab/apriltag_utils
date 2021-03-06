##

killall -9 gzserver
gnome-terminal --window -e 'bash -c "roslaunch px4 single_drone_payload_vision_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_command px4_multidrone_pos_estimator_pure_vision.launch uavID:=uav2; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_command px4_multidrone_pos_controller_gazebo.launch uavID:=uav2; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun track_april_tag april_tag_opencv_emulate; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun qt_ground_station qt_ground_station & rosrun px4_command set_uav2_mode; exec bash"' \

roslaunch optitrack_broadcast emulator_for_gazebo.launch > /dev/null 2>& 1 &
