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

float K1 = -4.5463;	//-64.1997/12; //-64.1997;
float K2 = -1.0975;	//-1.908;
float u = 0;

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;
float rollOffset;

float currentRollSpeed;
float speedOffset = -2.25;
float trueSpeed;

float correction;

int the_time = 0;

void read_Imu(sensor_msgs::Imu imu_msg)
{
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;

	//current roll angle, positif value in clockwise
	currentRoll = imu_msg.orientation.x;		
	currentRollSpeed = imu_msg.angular_velocity.x;
	
	//keep calibration after 5 seconds
	if(the_time < 15) 
	{
		rollOffset = currentRoll;
		ROS_INFO("Hold still for %d secondes: calibration %f",the_time, rollOffset);
	}
	currentRoll -= rollOffset;
	
}

int main(int argc, char **argv)
{	
	//argc = 0;
	//argv = 0;
	int freq = 50;
	
	/*******************************************/
	/* Definie LQR parameter */
	/*******************************************/
	if((atoi(argv[1])) != 0)
	{
		if((atoi(argv[1])) > -10  && (atoi(argv[1])) < 10 )
		K1 = atoi(argv[1]);
		
		if((atoi(argv[2])) > -10  && (atoi(argv[2])) < 10 )
		K2 = atoi(argv[2]);

	}
	
	ROS_INFO("Beginning with stabilisation with amplitudes: %f and %f", K1, K2);
	sleep(3);
							
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
	ros::Rate loop_rate(freq); // use to be a variable but now is imposed to 50Hz
	
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
	pilot.set_period(PILOT_PWM_OUT, freq);    //frequency 50Hz
	float pilot_input = PILOT_TRIM;

	sensor_msgs::Temperature rem_msg;
	sensor_msgs::Temperature ctrl_msg;
	
	int initTime = ros::Time::now().sec%1000;
	int i = 0;

	while (ros::ok())
	{
 
	  // indication degre to amplitude for pilot servo ; 90° = 900 microseconde --> 1° = 10 microsecondes
		trueSpeed = currentRollSpeed - speedOffset;
		u = K1*currentRoll + K2*trueSpeed;	//degree, positif values of imu in clockwise, currentRoll was already amputed by offset
		correction = u*10;
		if(the_time > 15) pilot_input = PILOT_TRIM + correction;			//rad->deg->amplitude pwm in ms
		
		//write readings on pwm output in miliseconds
		pilot.set_duty_cycle(PILOT_PWM_OUT, ((float)pilot_input)/1000.0f);
		
		//save values into msg container for the remote readings
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = 0; // motor_input;
		rem_msg.variance = pilot_input;
		
		i++;
		if(i == 25)		//added by Pascal, to get insgiht on the code and what is happening
		{
		ROS_INFO("Pilot: %f, Roll: %f, RollOffset: %f\n RollSpeed: %f, u: %f, correction %f", pilot_input, currentRoll, rollOffset, trueSpeed, u, correction);
		i=0;
		}
		//save values into msg container for the control readings
		ctrl_msg.header.stamp = ros::Time::now();
		ctrl_msg.temperature = 0; //speed;//_filt;
		ctrl_msg.variance = 0;//here it's supposed to be the control output

		// publish the messages
		remote_pub.publish(rem_msg);
		control_pub.publish(ctrl_msg);
		
		//Measure time for initial roll calibration
		the_time = ros::Time::now().sec%1000-initTime;

		ros::spinOnce();

		loop_rate.sleep();
	}

return 0;

}
