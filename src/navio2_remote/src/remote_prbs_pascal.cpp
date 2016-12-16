#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

#define MOTOR_PWM_OUT 9
#define SERVO_PWM_OUT 6
#define PILOT_PWM_OUT 3
//#define PRBS_FREQ 25 			//frequency of the prbs signal : 8 bit => min = 25/8 max = 25/1
#define SERVO_TRIM 1420.0f		//can be used to trim the steering if not done on the remote controller
#define PILOT_TRIM 1410.0f
#define PI 3.14159

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;
float rollOffset;			//calculated by the imu

int the_time = 0;

void read_Imu(sensor_msgs::Imu imu_msg)
{
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;

	//current roll angle
	currentRoll = imu_msg.orientation.x;
	//ROS_INFO("Time %d", the_time);			//Muted by Pascal

	//keep calibration after 15 seconds
	if(the_time < 15) 
	{
		rollOffset = currentRoll;
		ROS_INFO("Hold still for %d secondes: calibration %f",the_time, rollOffset);
	}
	
	currentRoll -= rollOffset ;
	//ROS_INFO("New Roll %f", currentRoll);	//Muted by Pascal
}

int main(int argc, char **argv)
{
	ROS_INFO("Start");
	int maxThrottle = 3000;		//max throttle in pwm, milisecondes
	int prbs_val = 0; 		//default prbs signal
	int freq = 50;
	int actuator = 0;		//to choose on which components to apply the prbs signal, pilot or servo
	int PRBS_FREQ = 25;
	
	ROS_INFO("number of argc %d", argc);
	
	prbs_val = atoi(argv[1]);			//prbs_val is the pwm apmilutude of the prbs signal on the pilot
	if(prbs_val > 450 || prbs_val < 0)
	{
		ROS_INFO("amp prbs must be between 0 and 450");
		return 0;
	}

	PRBS_FREQ = atoi(argv[2]);			//prbs_val is the pwm apmilutude of the prbs signal on the pilot
	if(PRBS_FREQ > 100 || PRBS_FREQ < 0)
	{
		ROS_INFO("prbs freq must be between 0 and 100");
		return 0;
	}
	
	freq = atoi(argv[3]);
	if(freq <= 0 )
	{
		ROS_INFO("Frequency must be more than 0");
		return 0;
	}
	
	if(atoi(argv[4]) < maxThrottle) 
		maxThrottle = atoi(argv[4]);
	
	actuator = atoi(argv[5]);
	if(actuator != 1 && actuator != 0)
		return 0;

	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "remote_reading_handler");
	ros::NodeHandle n;
	//ros::Publisher steering_pub = n.advertise<sensor_msgs::Temperature>("steering_readings", 1000);
	//ros::Publisher pilot_pub = n.advertise<sensor_msgs::Temperature>("pilot_readings", 1000);
	//ros::Publisher speed_pub = n.advertise<sensor_msgs::Temperature>("speed_readings", 1000);
	
	//subscribe to imu topic
	ros::Subscriber imu_sub = n.subscribe("imu_readings", 1000, read_Imu);

	//running rate = freq Hz
	ros::Rate loop_rate(freq);
	
	/*******************************************/
	/* Initialize the RC input, and PWM output */
	/*******************************************/

	RCInput rcin;
	rcin.init();
	PWM servo;
	PWM motor;
	PWM pilot;

	if (!motor.init(MOTOR_PWM_OUT)) {
		fprintf(stderr, "Motor Output Enable not set. Are you root?\n");
		return 0;
    	}

	if (!pilot.init(PILOT_PWM_OUT)) {
		fprintf(stderr, "Pilot Output Enable not set. Are you root?\n");
		return 0;
    	}
	
	if (!servo.init(SERVO_PWM_OUT)) {
		fprintf(stderr, "Servo Output Enable not set. Are you root?\n");
		return 0;
    	}

	motor.enable(MOTOR_PWM_OUT);
	pilot.enable(PILOT_PWM_OUT);
	servo.enable(SERVO_PWM_OUT);

	motor.set_period(MOTOR_PWM_OUT, 50); //frequency 50Hz
	pilot.set_period(PILOT_PWM_OUT, 50);
	servo.set_period(SERVO_PWM_OUT, 50); 

	int motor_input = 0;
	int pilot_input = 0;
	int servo_input = 0;
	
	sensor_msgs::Temperature steering_msg;
	sensor_msgs::Temperature pilot_msg;
	sensor_msgs::Temperature speed_msg;
	
	//prbs start state
	int start_state = 0x7D0;
	int lfsr = start_state;

	int prbs_low = -prbs_val;
	int prbs_high = prbs_val;
	int amp_prbs = prbs_low;
	
	float pilotRoll = 0;

	//speed in m/s
	float speed = 0;
	float speed_filt = 0;
	int dtf = 0;// dtf read from arduino. dtf = dt*4 in msec
	float R = 0.064f; //Rear Wheel Radius D = 1280mm

	int ctr = 0; //counter for the period divider
	int i = 0;	//coutner for information display
	
	int initTime = ros::Time::now().sec%1000;
	
	while (ros::ok())
	{
		ctr %= freq/PRBS_FREQ;

		//Throttle saturation
		if(rcin.read(3) >= maxThrottle)
			motor_input = maxThrottle;
		else
			motor_input = rcin.read(3);

		//pilot control with prbs
		if(!ctr)
		{
			int bit = ((lfsr >> 0) ^ (lfsr >> 2)) & 1;
			lfsr = (lfsr >> 1) | (bit << 8); //was bit << 10 before

			if (bit == 1)
				amp_prbs = prbs_high;
			else if (bit == 0)
				amp_prbs = prbs_low;
		}
		ctr++;
		
		switch ( actuator ) 
		{
			case 0:	//means pilot is prbs
				//Pilot steering
				if(the_time > 15) pilot_input = PILOT_TRIM + amp_prbs; //to avoid moving during calibartion
				pilotRoll = (amp_prbs)/10; 		//pwm amplitude -> deg
				
				//Servo steering
				servo_input = rcin.read(2) - 1500 + SERVO_TRIM;
				
				// In case user has to make a curve to avoid an obstacle
				if (servo_input > SERVO_TRIM + 50 || servo_input < SERVO_TRIM  - 50)		
					pilot_input = PILOT_TRIM;
			break;
			case 1: //means servo is prbs
				pilot_input = PILOT_TRIM;
				if(the_time > 15) servo_input =rcin.read(2) - 1500 + SERVO_TRIM + amp_prbs; //to avoid moving during calibartion
				
				// In case user has to make a curve to avoid an obstacle
				if (servo_input > SERVO_TRIM + prbs_val + 50 || servo_input < SERVO_TRIM - prbs_val - 50)		
					servo_input = rcin.read(2) - 1500 + SERVO_TRIM;
			  break;
			default:
			ROS_INFO("Error, bad input, quitting\n");
				return 0;
			  break;
		}
				
		//write readings on pwm output
		motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
		pilot.set_duty_cycle(PILOT_PWM_OUT, ((float)pilot_input)/1000.0f);
		servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);
		

		dtf = rcin.read(5)-1000;				//Pascal: signal from the hall sensor
		speed = 4.0f*PI*R*1000.0f/((float)dtf);
		if(speed < 0 || dtf < 40) speed = 0;
		
		// low pass filtering of the speed with tau = 0.1
		float alpha = 0.01f/(0.01f+0.1f);
		speed_filt = alpha*speed + (1.0f-alpha)*speed_filt;
		
		//debug info
		//ROS_INFO("Thrust usec = %d    ---   Steering usec = %d", motor_input, servo_input);
		//ROS_INFO("dtf msec = %d    ---   Speed m/s = %f", dtf, speed);
		//printf("[Thrust:%d] - [Pilot:%d] - [dtf:%4d] - [Speed:%2.2f]\n", motor_input, pilot_input, dtf, speed_filt);
		//printf("rcin %d  %d  %d  %d  %d  %d  %d  %d\n",rcin.read(0), rcin.read(1), rcin.read(2), rcin.read(3), rcin.read(4), rcin.read(5), rcin.read(6), rcin.read(7));
		i++;
		if(i == freq/2)		//To get insgiht on the code and what is happening
		{
		ROS_INFO("Current Roll: %f, Pilot Roll: %f, Steering: %d,\n Time: %i, Throttle: %d and Speed: %f", currentRoll, pilotRoll, servo_input, the_time, motor_input, speed);
		i=0;
		}
					
		//save values into msg container for the remote readings
		steering_msg.header.stamp = ros::Time::now();
		steering_msg.temperature = servo_input;
		steering_msg.variance = currentRoll;

		//save values into msg container for the control readings
		pilot_msg.header.stamp = ros::Time::now();
		pilot_msg.temperature = pilotRoll;
		pliot_msg.variance = currentRoll;
		
		//save values into msg container for the remote readings
		speed_msg.header.stamp = ros::Time::now();
		speed_msg.temperature = motor_input;
		speed_msg.variance = speed;
		
		//publish the messages
		//steering_pub.publish(steering_msg);
		//pilot_pub.publish(pilot_msg);
		//speed_pub.publish(speed_msg);
		
		//Measure time for initial roll calibration
		the_time = ros::Time::now().sec%1000-initTime;

		ros::spinOnce();		//Call this function to allow ROS to process incoming messages 

		loop_rate.sleep();		//Sleep for the rest of the cycle, to enforce the loop rate

	}


return 0;

}
