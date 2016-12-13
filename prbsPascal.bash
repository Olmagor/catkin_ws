#!/bin/bash
SESSION=$USER

#no log
if [ "$#" -eq 4 ]
then
    echo "Here we go in IDENTIFICATION PRBS MODE"
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
    tmux send-keys "rosrun navio2_imu imu_pub $(($2-1))" C-m
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_prbs_pascal $1 $2 $3 $4" C-m
elif [ "$#" -eq 5 ] && [ "$5" == '-log' ]          #to active the writing of the log files
  then
      echo "Here we go in IDENTIFICATION PRBS MODE, WITH log files"
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
      tmux send-keys "rosrun navio2_imu imu_pub $(($2-1))" C-m
      tmux select-pane -t 0
      tmux split-window -v
      tmux send-keys "sleep 5" C-m
      tmux send-keys "sudo -i" C-m
      tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
      tmux send-keys "rosrun navio2_remote remote_prbs_pascal $1 $2 $3" C-m
      tmux split-window -v
      tmux send-keys "sleep 10" C-m                                       #no needs to start earlier because 10/15 sec of calibration
      tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
      tmux send-keys "cd /home/pi/bagfiles" C-m
      tmux send-keys "rosbag record -a" C-m
else
    echo "Usage :   ./rosCustom.bash [prbs amp] [freq prbs] [freq ros] [MaxThrottlePwm] [-log], -log is not necessary" #MaxThrottlePwm is limited to 2000 in the cpp
    echo "[prbs amp] can be a value between 0 and 450 but best between 0 and 100, [prbs freq] must be a positive value usually between 20-100hz"
    echo "[freq ros] must be a positive value usually 50Hz, [MaxThrottlePwm] can be a value between 1500-2000ms"
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
