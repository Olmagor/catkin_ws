#!/bin/bash
SESSION=$USER

#no log
if [ "$#" -eq 1 ] && [ "$1" == 'auto' ]
then
    echo "Here we go in AUTO MODE, freq= 50, MaxThrottlePwm=2000, Kp=0.7, Ki=0.7, Kd=0"
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
    tmux send-keys "rosrun navio2_remote remote_multiCtr 50 2000 0.7 0.7 0" C-m
elif [ "$#" -eq 5 ]
then
    echo "Here we go WITHOUT log files"
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
    tmux send-keys "rosrun navio2_imu imu_pub $(($1-1))" C-m
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_multiCtr $1 $2 $3 $4 $5" C-m
    elif [ "$#" -eq 6 ] && [ "$6" == '-log' ]          #to active the writing of the log files
    then
        echo "Here we go WITH log files"
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
        tmux send-keys "rosrun navio2_imu imu_pub $(($1-1))" C-m
        tmux select-pane -t 0
        tmux split-window -v
        tmux send-keys "sleep 5" C-m
        tmux send-keys "sudo -i" C-m
        tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
        tmux send-keys "rosrun navio2_remote remote_multiCtr $1 $2 $3 $4 $5" C-m
        tmux split-window -v
        tmux send-keys "sleep 10" C-m                                       #no needs to start earlier because 10/15 sec of calibration
        tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
        tmux send-keys "cd /home/pi/bagfiles" C-m
        tmux send-keys "rosbag record -a" C-m
else
    echo "Usage :   ./rosCustom.bash [freq] [MaxThrottlePwm] [Kp] [Ki] [Kd] [-log], -log is not necessary" #MaxThrottlePwm is limited to 2000 in the cpp
    echo "Or :      ./rosCustom.bash auto : for automatic mode i.e. freq= 50, MaxThrottlePwm=2000, Kp=0.7, Ki=0.7, Kd=0"
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
