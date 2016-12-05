#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include <sstream>

#define PILOT_PWM_OUT 3
#define PI 3.14159

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;

float currentSpeed;
ros::Time currentTimeSpeed;
ros::Time previousTimeSpeed;

int main(int argc, char **argv)
{
	ROS_INFO("Start");

	ROS_INFO("Beginning with prbs : %d frequency %d, and saturation  : %d", prbs_val, freq, saturation);

 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "remote_reading_handler");
	ros::NodeHandle n;
	ros::Publisher remote_pub = n.advertise<sensor_msgs::Temperature>("remote_readings", 1000);
	ros::Publisher control_pub = n.advertise<sensor_msgs::Temperature>("control_readings", 1000);
  
  //subscribe to imu topic
	ros::Subscriber imu_sub = n.subscribe("imu_readings", 1000, read_Imu);

	
	//running rate = freq Hz
	ros::Rate loop_rate(50); // used to be a variable but now is imposed to 50Hz
	
	/*******************************************/
	/* Initialize the RC input, and PWM output */
	/*******************************************/

	RCInput rcin;
	rcin.init();
	PWM pilot;

	if (!pilot.init(PILOT_PWM_OUT)) {
		fprintf(stderr, "Pilot Output Enable not set. Are you root?\n");
		return 0;
    	}
	
	pilot.enable(PILOT_PWM_OUT);

	pilot.set_period(PILOT_PWM_OUT, 50);    //frequency 50Hz

	int pilot_input = 0;

	sensor_msgs::Temperature rem_msg;
	sensor_msgs::Temperature ctrl_msg;


	//speed in m/s
	float speed = 0;
	float speed_filt = 0;
	int dtf = 0;// dtf read from arduino. dtf = dt*4 in msec
	float R = 0.0625f; //Rear Wheel Radius

	int ctr = 0; //counter for the period divider 
	while (ros::ok())
	{
  
  void read_Imu(sensor_msgs::Imu imu_msg)
{
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;

	//current roll angle
	currentRoll = imu_msg.orientation.x;
}
	  // indication degre to amplitude for pilot servo ; 90° = 900 microseconde --> 1° = 0.1ms
		
		//write readings on pwm output in miliseconds
		pilot.set_duty_cycle(PILOT_PWM_OUT, ((float)pilot_input)/1000.0f);
		
		//save values into msg container for the remote readings
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = 0; // motor_input;
		rem_msg.variance = pilot_input;
		
		//debug info
		//ROS_INFO("Thrust usec = %d    ---   Steering usec = %d", motor_input, servo_input);
		//ROS_INFO("dtf msec = %d    ---   Speed m/s = %f", dtf, speed);
		//printf("[Thrust:%d] - [Pilot:%d] - [dtf:%4d] - [Speed:%2.2f]\n", motor_input, pilot_input, dtf, speed_filt);
		//printf("rcin %d  %d  %d  %d  %d  %d  %d  %d\n",rcin.read(0), rcin.read(1), rcin.read(2), rcin.read(3), rcin.read(4), rcin.read(5), rcin.read(6), rcin.read(7));
		ROS_INFO("Pilot: %d", pilot_input);

		//save values into msg container for the control readings
		ctrl_msg.header.stamp = ros::Time::now();
		ctrl_msg.temperature = 0; //speed;//_filt;
		ctrl_msg.variance = 0;//here it's supposed to be the control output

		// publish the messages
		remote_pub.publish(rem_msg);
		control_pub.publish(ctrl_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


return 0;

}
