#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include <sstream>

#define MOTOR_PWM_OUT 9
#define SERVO_PWM_OUT 6
#define PILOT_PWM_OUT 3
#define PRBS_FREQ 25 			//frequency of the prbs signal : 8 bit => min = 25/8 max = 25/1
#define SERVO_TRIM 1430.0f		//can be used to trim the steering if not done on the remote controller
#define PILOT_TRIM 1410.0f

#define PI 3.14159

int main(int argc, char **argv)
{
	ROS_INFO("Start");
	int saturation = 2000;
	int prbs_val = 0; 		//default prbs signal
	int freq = 100;

	ROS_INFO("number of argc %d", argc);
	
	prbs_val = atoi(argv[1]);
	if(prbs_val > 500 || prbs_val < 0)
	{
		ROS_INFO("prbs val must be between 0 and 500");
		return 0;
	}

	if(atoi(argv[2]) > 0 )
		freq = atoi(argv[2]);
	else
	{
		ROS_INFO("Frequency must be more than 0");
		return 0;
	}

	if(atoi(argv[3]) > 2000) saturation = 2000;
	else saturation = atoi(argv[3]);

	ROS_INFO("Beginning with prbs : %d frequency %d, and saturation  : %d", prbs_val, freq, saturation);


 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "remote_reading_handler");
	ros::NodeHandle n;
	ros::Publisher remote_pub = n.advertise<sensor_msgs::Temperature>("remote_readings", 1000);
	ros::Publisher control_pub = n.advertise<sensor_msgs::Temperature>("control_readings", 1000);
	
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
	
	int servo_input_0 = rcin.read(2);

	sensor_msgs::Temperature rem_msg;
	sensor_msgs::Temperature ctrl_msg;

	//prbs start state
	int start_state = 0x7D0;
	int lfsr = start_state;

	int prbs_low = PILOT_TRIM - prbs_val;
	int prbs_high = PILOT_TRIM + prbs_val;
	int pilot_prbs = prbs_low;

	//speed in m/s
	float speed = 0;
	float speed_filt = 0;
	int dtf = 0;// dtf read from arduino. dtf = dt*4 in msec
	float R = 0.0625f; //Rear Wheel Radius

	int ctr = 0; //counter for the period divider 
	while (ros::ok())
	{
		ctr %= freq/PRBS_FREQ;

		//Throttle saturation
		if(rcin.read(3) >= saturation)
			motor_input = saturation;
		else
			motor_input = rcin.read(3);

		//pilot control with prbs
		if(!ctr)
		{
			int bit = ((lfsr >> 0) ^ (lfsr >> 2)) & 1;
			lfsr = (lfsr >> 1) | (bit << 8); //was bit << 10 before

			if (bit == 1)
				pilot_prbs = prbs_high;
			else if (bit == 0)
				pilot_prbs = prbs_low;
		}
		ctr++;
		pilot_input = pilot_prbs;
		
		//Servo steering
		servo_input = rcin.read(2);//-1500 + SERVO_TRIM;		
			
		// In case user has to make a curve to avoid an obstacle
		//if (servo_input > SERVO_TRIM + 50 || servo_input < SERVO_TRIM  - 50)		
			pilot_input = PILOT_TRIM;
		
		//write readings on pwm output
		motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
		pilot.set_duty_cycle(PILOT_PWM_OUT, ((float)pilot_input)/1000.0f);
		servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);
		
		//save values into msg container for the remote readings
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = motor_input;
		rem_msg.variance = pilot_input;

		dtf = rcin.read(4)-1000;				//Pascal: signal from the hall sensor
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
		ROS_INFO("Throttle :%d, Pilot: %d, Steering: %d, Hall sensor: %i and Speed: %f", motor_input, pilot_input, servo_input, dtf, speed);

		//save values into msg container for the control readings
		ctrl_msg.header.stamp = ros::Time::now();
		ctrl_msg.temperature = speed;//_filt;
		ctrl_msg.variance = 0;//here it's supposed to be the control output

		// publish the messages
		remote_pub.publish(rem_msg);
		control_pub.publish(ctrl_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


return 0;

}
