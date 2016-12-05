#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

#define PILOT_PWM_OUT 3
#define PILOT_TRIM 1410.0f
#define PI 3.14159

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;

float currentRollSpeed

void read_Imu(sensor_msgs::Imu imu_msg)
{
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;

	//current roll angle, positif value in clockwise
	currentRoll = imu_msg.orientation.x;		
	currentRollSpeed = imu.angular_velocity.x;
}

int main(int argc, char **argv)
{
	ROS_INFO("Beginning with stabilisation");
	
	/*******************************************/
	/* Definie LQR parameter */
	/*******************************************/
	float K1=-64.1997;
	float K2=-6.4701;
	float u;
	
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
	ros::Rate loop_rate(50); // use to be a variable but now is imposed to 50Hz
	
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

	while (ros::ok())
	{
 
	  // indication degre to amplitude for pilot servo ; 90° = 900 microseconde --> 1° = 10 microsecondes
		u = K1*currentRoll + K2*currentRollSpeed;	//rad, positif values of imu in clockwise
		pilot_input = PILOT_TRIM + (u*180/PI)*10;			//rad->deg->amplitude pwm in ms
		
		//write readings on pwm output in miliseconds
		pilot.set_duty_cycle(PILOT_PWM_OUT, ((float)pilot_input)/1000.0f);
		
		//save values into msg container for the remote readings
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = 0; // motor_input;
		rem_msg.variance = pilot_input;
		
		ROS_INFO("Pilot: %d, Roll : %f and RollSpeed : %f", pilot_input, currentRoll, currentRollSpeed);

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
