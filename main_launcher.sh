#!/bin/bash

NUMID_DRONE=111
DRONE_SWARM_ID=1
MAV_NAME=hummingbird_adr
export AEROSTACK_PROJECT=${AEROSTACK_STACK}/projects/smach_mission_gazebo

. ${AEROSTACK_STACK}/config/mission/setup.sh
#---------------------------------------------------------------------------------------------
# INTERNAL PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Motor speed controller                                                                      ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Motor speed controller"  --command "bash -c \"
roslaunch motor_speed_controller motor_speed_controller.launch --wait \
  namespace:=drone$NUMID_DRONE \
  mav_name:=hummingbird;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Gazebo Interface                                                                            ` \
`#---------------------------------------------------------------------------------------------` \
  --tab --title "Gazebo Interface" --command "bash -c \"
roslaunch gazebo_interface gazebo_interface.launch --wait \
    robot_namespace:=drone$NUMID_DRONE \
    drone_id:=$DRONE_SWARM_ID \
    mav_name:=$MAV_NAME;
exec bash\""  &
gnome-terminal \
`#---------------------------------------------------------------------------------------------` \
`# Basic Quadrotor Behaviors                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Basic Quadrotor Behaviors" --command "bash -c \"
roslaunch basic_quadrotor_behaviors basic_quadrotor_behaviors.launch --wait \
  namespace:=drone$NUMID_DRONE;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Quadrotor Motion With PID Control                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Quadrotor Motion With PID Control" --command "bash -c \"
roslaunch quadrotor_motion_with_pid_control quadrotor_motion_with_pid_control.launch --wait \
    namespace:=drone$NUMID_DRONE \
    robot_config_path:=${AEROSTACK_PROJECT}/configs/drone$NUMID_DRONE \
    uav_mass:=0.7;
exec bash\""  &

sleep 3






