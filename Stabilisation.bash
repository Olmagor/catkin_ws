#!/bin/bash
SESSION=$USER

#no log
if [ "$#" -eq 2 ]
then
    echo "Here we go in STABILISATION MODE"
    sleep 2
    tmux -2 new-session -d -s $SESSION
    tmux new-window -t $SESSION:1 -n 'ROS'
    tmux split-window -h
    tmux select-pane -t 0
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "roscore" C-m
    tmux select-pane -t 1
    tmux send-keys "sleep 5" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_imu imu_pub 49" C-m
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_stabilisation $1 $2" C-m
elif [ "$#" -eq 3 ] && [ "$3" == '-log' ]          #to active the writing of the log files
  then
      echo "Here we go in STABILISATION MODE, WITH log files"
      sleep 2
      tmux -2 new-session -d -s $SESSION
      tmux new-window -t $SESSION:1 -n 'ROS'
      tmux split-window -h
      tmux select-pane -t 0
      tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
      tmux send-keys "roscore" C-m
      tmux select-pane -t 1
      tmux send-keys "sleep 5" C-m
      tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
      tmux send-keys "rosrun navio2_imu imu_pub 49" C-m
      tmux select-pane -t 0
      tmux split-window -v
      tmux send-keys "sleep 5" C-m
      tmux send-keys "sudo -i" C-m
      tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
      tmux send-keys "rosrun navio2_remote remote_stabilisation $1" C-m
      tmux split-window -v
      tmux send-keys "sleep 5" C-m                                       #no needs to start earlier because no calibration
      tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
      tmux send-keys "cd /home/pi/bagfiles" C-m
      tmux send-keys "rosbag record -a" C-m
else
    echo "Usage :   ./stabilisation.bash [K1] [k2] [-log], -log is not necessary" 
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
