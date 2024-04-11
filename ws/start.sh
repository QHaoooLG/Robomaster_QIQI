#!/bin/bash



gnome-terminal --tab -x bash -c "\
   source /opt/ros/humble/setup.bash;\
   source /home/w/ws/install/setup.bash;\
   ros2 launch rm_vision_bringup vision_bringup.launch.py ;\
exec bash"

#gnome-terminal --tab -x bash -c "\
#   sleep 0.2;\
#   source /opt/ros/humble/setup.bash;\
#   source /home/w/ws/install/setup.bash;\
#   ros2 run foxglove_bridge foxglove_bridge;\
   
#exec bash"


#gnome-terminal --tab -x bash -c "\
#sleep 0.2; \                                                  
# source /opt/ros/humble/setup.bash 
#rviz2 ; \
#exec bash"


