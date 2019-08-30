cd;
cd deepclaw_ws;
source devel/setup.bash;
roslaunch denso_robot_bringup cobotta_bringup.launch sim:=false robot_address:=192.168.0.1;