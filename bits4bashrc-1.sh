
# Here EGB349 bash.rc tools nice to have in every new terminal

source /opt/ros/noetic/setup.bash
catkin_ws="EGH450G3/ros_ws"

source ~/${catkin_ws}/devel/setup.bash

disros() {
  # Setup for distributed ROS
  export ROS_IP="$(hostname -I | cut -d' ' -f1)"
  echo "Identifying as: $ROS_IP"
}

# Start by default with as master
disros

# Activate multiple terminals
function run349 (){
    echo "Starting Tmux session for EGB349 UAV control"
    sleep 2
    tmux new-session -s UAVTeam3\; set -g mouse on\; send-keys "roscore" C-m\; split-window -h -p 85\; select-pane -t 0\; split-window -v\;                   \
        send-keys "htop" C-m\; select-pane -t 2\; split-window -h\; select-pane -t 2\;                   \
        send-keys "ls -l" C-m\; split-window -v -p 75\;               \
        send-keys "sleep 5; roslaunch /${catkin_ws}/launch/control.launch" C-m\; split-window -v -p 60\;                                 \
        send-keys "sleep 10; roslaunch qutas_lab_450 environment.launch" C-m\; split-window -v \;                                        \
        send-keys "sleep 10; Vision code here... C-m" \;  select-pane -t 6\;                                   \
        send-keys "rosbag -a" \;  split-window -v -p 85\;                      \
        send-keys "rostopic echo /mavros/local_position/pose" C-m\; split-window -v -p 80\; select-pane -t 7\; split-window -h\; \
        send-keys "rostopic echo /mavros/vision_position/pose" C-m\; select-pane -t 9\;                                                 \
        send-keys "" \; split-window -v -p 10\;                             \
        send-keys "tmux kill-session" \;
}
export -f run349
