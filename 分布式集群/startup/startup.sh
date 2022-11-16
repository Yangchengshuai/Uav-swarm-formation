gnome-terminal -x bash -c "source $HOME/mavros_ws/devel/setup.bash;roslaunch px4_realsense_bridge bridge.launch;exec bash"  
sleep 10s  
gnome-terminal -x bash -c  "roslaunch mavros px4.launch;exec bash"  
sleep 3s  
gnome-terminal -x bash -c  "cd /home/unionsys/Desktop;python follower_all.py;exec bash"  
sleep 2s  
gnome-terminal -x bash -c  "cd /home/unionsys/Desktop;python Yfollower1_300-3.py;exec bash"
wait  
exit 0  
