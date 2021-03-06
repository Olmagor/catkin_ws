#include "RCInput.h"
#include "PWM.h"
#include "Util.h"
//#include "RGBled.h"
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Imu.h"
#include <sstream>

#define MOTOR_PWM_OUT 9
#define SERVO_PWM_OUT 0

//Maximum Integration angle
#define MAX_IERR 4
#define PI 3.14159
#define MAX_ROLL_ANGLE 10.0f
#define SERVO_TRIM 1440.0f

// PID for roll angle
#define Kp 0.3f
#define Ki 0.0f
#define Kd 0.03f

#define MAX_IERR_MOTOR 20.6

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;

float currentSpeed;
ros::Time currentTimeSpeed;
ros::Time previousTimeSpeed;

float err;
float derr;
float Kierr;

float err_m;
float derr_m;
float Kierr_m;

//parameters to pass
float Kp_m;
float Ki_m;
float Kd_m;

float RollOffset = 0; // offset to add to roll measurement for initial calibration

int the_time = 0;

// lookup table for the 8 shape with 0.02 sampling time
float lookup_roll[] = { 0.00, 0.02, 0.04, 0.08, 0.11, 0.15, 0.18, 0.22, 0.25, 0.29, 0.33, 0.36, 0.40, 0.44, 0.47, 0.51, 0.55, 0.59, 0.63, 0.67, 0.71, 0.75, 0.79, 0.83, 0.87, 0.91, 0.96, 1.00, 1.04, 1.09, 1.14, 1.18, 1.23, 1.28, 1.33, 1.38, 1.43, 1.48, 1.54, 1.59, 1.65, 1.71, 1.77, 1.83, 1.89, 1.96, 2.02, 2.09, 2.16, 2.23, 2.31, 2.38, 2.46, 2.54, 2.63, 2.71, 2.80, 2.90, 2.99, 3.09, 3.19, 3.30, 3.41, 3.53, 3.65, 3.77, 3.90, 4.03, 4.17, 4.32, 4.47, 4.63, 4.79, 4.97, 5.15, 5.34, 5.53, 5.74, 5.96, 6.18, 6.42, 6.67, 6.93, 7.21, 7.50, 7.80, 8.12, 8.45, 8.81, 9.18, 9.57, 9.98, 10.41, 10.86, 11.34, 11.84, 12.37, 12.92, 13.49, 14.09, 14.71, 15.36, 16.03, 16.72, 17.43, 18.16, 18.89, 19.64, 20.38, 21.12, 21.84, 22.53, 23.20, 23.82, 24.40, 24.91, 25.36, 25.72, 26.00, 26.19, 26.29, 26.30, 26.20, 26.02, 25.76, 25.41, 24.99, 24.51, 23.97, 23.38, 22.76, 22.12, 21.45, 20.78, 20.10, 19.42, 18.75, 18.09, 17.45, 16.82, 16.22, 15.64, 15.08, 14.54, 14.03, 13.54, 13.07, 12.63, 12.20, 11.80, 11.43, 11.07, 10.73, 10.40, 10.10, 9.81, 9.54, 9.28, 9.04, 8.81, 8.59, 8.39, 8.19, 8.01, 7.84, 7.68, 7.52, 7.38, 7.25, 7.12, 7.00, 6.89, 6.79, 6.69, 6.60, 6.52, 6.44, 6.36, 6.30, 6.24, 6.18, 6.13, 6.09, 6.05, 6.01, 5.98, 5.95, 5.93, 5.92, 5.90, 5.90, 5.89, 5.90, 5.90, 5.91, 5.93, 5.95, 5.97, 6.00, 6.03, 6.07, 6.12, 6.17, 6.22, 6.28, 6.34, 6.42, 6.49, 6.57, 6.66, 6.76, 6.86, 6.97, 7.09, 7.21, 7.35, 7.49, 7.64, 7.80, 7.97, 8.15, 8.34, 8.55, 8.76, 8.99, 9.23, 9.49, 9.76, 10.05, 10.36, 10.68, 11.02, 11.38, 11.76, 12.16, 12.59, 13.03, 13.50, 13.99, 14.51, 15.05, 15.62, 16.21, 16.82, 17.45, 18.10, 18.76, 19.44, 20.12, 20.81, 21.49, 22.16, 22.81, 23.43, 24.01, 24.55, 25.03, 25.44, 25.78, 26.05, 26.22, 26.30, 26.28, 26.18, 25.98, 25.70, 25.32, 24.87, 24.36, 23.79, 23.17, 22.51, 21.81, 21.10, 20.37, 19.64, 18.90, 18.18, 17.46, 16.76, 16.08, 15.41, 14.77, 14.16, 13.56, 12.99, 12.45, 11.93, 11.43, 10.95, 10.50, 10.07, 9.66, 9.27, 8.90, 8.55, 8.21, 7.89, 7.59, 7.30, 7.02, 6.76, 6.51, 6.27, 6.04, 5.82, 5.62, 5.42, 5.23, 5.05, 4.87, 4.71, 4.55, 4.39, 4.25, 4.10, 3.97, 3.84, 3.71, 3.59, 3.48, 3.36, 3.26, 3.15, 3.05, 2.95, 2.86, 2.77, 2.68, 2.60, 2.51, 2.43, 2.36, 2.28, 2.21, 2.14, 2.07, 2.00, 1.94, 1.87, 1.81, 1.75, 1.69, 1.64, 1.58, 1.52, 1.47, 1.42, 1.37, 1.32, 1.27, 1.22, 1.17, 1.13, 1.08, 1.03, 0.99, 0.95, 0.90, 0.86, 0.82, 0.78, 0.74, 0.70, 0.66, 0.62, 0.58, 0.54, 0.51, 0.47, 0.43, 0.39, 0.36, 0.32, 0.28, 0.25, 0.21, 0.18, 0.14, 0.11, 0.07, 0.03, -0.00, -0.04, -0.07, -0.11, -0.14, -0.18, -0.22, -0.25, -0.29, -0.32, -0.36, -0.40, -0.43, -0.47, -0.51, -0.55, -0.59, -0.62, -0.66, -0.70, -0.74, -0.78, -0.83, -0.87, -0.91, -0.95, -1.00, -1.04, -1.09, -1.13, -1.18, -1.23, -1.28, -1.32, -1.38, -1.43, -1.48, -1.53, -1.59, -1.65, -1.70, -1.76, -1.82, -1.89, -1.95, -2.02, -2.09, -2.16, -2.23, -2.30, -2.38, -2.46, -2.54, -2.62, -2.71, -2.80, -2.89, -2.99, -3.08, -3.19, -3.29, -3.40, -3.52, -3.64, -3.76, -3.89, -4.02, -4.16, -4.31, -4.46, -4.62, -4.78, -4.95, -5.13, -5.32, -5.52, -5.72, -5.94, -6.17, -6.40, -6.65, -6.91, -7.19, -7.47, -7.78, -8.09, -8.43, -8.78, -9.15, -9.54, -9.95, -10.38, -10.83, -11.30, -11.80, -12.33, -12.88, -13.45, -14.05, -14.67, -15.31, -15.98, -16.67, -17.38, -18.11, -18.84, -19.58, -20.33, -21.06, -21.79, -22.48, -23.15, -23.78, -24.36, -24.88, -25.33, -25.70, -25.98, -26.18, -26.29, -26.30, -26.21, -26.04, -25.78, -25.44, -25.02, -24.54, -24.01, -23.43, -22.81, -22.17, -21.50, -20.83, -20.15, -19.47, -18.80, -18.14, -17.49, -16.87, -16.26, -15.68, -15.11, -14.58, -14.06, -13.57, -13.10, -12.66, -12.23, -11.83, -11.45, -11.09, -10.75, -10.43, -10.12, -9.83, -9.56, -9.30, -9.05, -8.82, -8.60, -8.40, -8.21, -8.02, -7.85, -7.69, -7.54, -7.39, -7.26, -7.13, -7.01, -6.90, -6.79, -6.70, -6.61, -6.52, -6.44, -6.37, -6.30, -6.24, -6.19, -6.14, -6.09, -6.05, -6.01, -5.98, -5.96, -5.94, -5.92, -5.91, -5.90, -5.89, -5.90, -5.90, -5.91, -5.93, -5.95, -5.97, -6.00, -6.03, -6.07, -6.11, -6.16, -6.22, -6.27, -6.34, -6.41, -6.49, -6.57, -6.66, -6.75, -6.85, -6.96, -7.08, -7.20, -7.34, -7.48, -7.63, -7.79, -7.96, -8.14, -8.33, -8.53, -8.75, -8.97, -9.22, -9.47, -9.74, -10.03, -10.33, -10.66, -11.00, -11.35, -11.73, -12.13, -12.56, -13.00, -13.47, -13.96, -14.47, -15.01, -15.58, -16.16, -16.77, -17.40, -18.05, -18.71, -19.39, -20.07, -20.76, -21.44, -22.11, -22.76, -23.39, -23.97, -24.52, -25.00, -25.42, -25.76, -26.03, -26.21, -26.29, -26.29, -26.19, -26.00, -25.72, -25.35, -24.91, -24.40, -23.83, -23.21, -22.55, -21.86, -21.15, -20.42, -19.69, -18.96, -18.23, -17.51, -16.81, -16.13, -15.46, -14.82, -14.20, -13.60, -13.03, -12.49, -11.96, -11.46, -10.99, -10.53, -10.10, -9.69, -9.30, -8.93, -8.57, -8.24, -7.91, -7.61, -7.32, -7.04, -6.78, -6.53, -6.29, -6.06, -5.84, -5.63, -5.43, -5.24, -5.06, -4.88, -4.72, -4.56, -4.40, -4.26, -4.11, -3.98, -3.85, -3.72, -3.60, -3.48, -3.37, -3.26, -3.16, -3.06, -2.96, -2.87, -2.78, -2.69, -2.60, -2.52, -2.44, -2.36, -2.29, -2.21, -2.14, -2.07, -2.01, -1.94, -1.88, -1.82, -1.76, -1.70, -1.64, -1.58, -1.53, -1.47, -1.42, -1.37, -1.32, -1.27, -1.22, -1.18, -1.13, -1.08, -1.04, -0.99, -0.95, -0.91, -0.86, -0.82, -0.78, -0.74, -0.70, -0.66, -0.62, -0.58, -0.55, -0.51, -0.47, -0.43, -0.40, -0.36, -0.32, -0.29, -0.25, -0.21, -0.18, -0.14, -0.11, -0.07 };

int pid_Servo_Output(int desired_roll)
{//	ROS_INFO("pid servo");
	//calculate errors
	float previousErr = err;
	
	err = desired_roll - currentRoll;

	long timeNow = currentTime.nsec;

	//time between now and last roll message we got
	double dTnsec = (timeNow - previousTime.nsec);///(10e9f); //in sec
	if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
	double dT = dTnsec/(1e9f);

	if(dT > 0)
		derr = (err - previousErr)/dT;

	Kierr += Ki*err*dT;

	//old anti wind-up (saturation)
	if(Kierr > MAX_IERR) Kierr = MAX_IERR;
	if(Kierr < -MAX_IERR) Kierr = -MAX_IERR;
	
	//PID CONTROLLER
	float controlSignal = Kp*err + Kierr + Kd*derr; // should be between +- 22 deg
	
	int pwmSignal = (int)((-controlSignal*250.0f)/22.0f)+(SERVO_TRIM);
	if(pwmSignal > 1750) pwmSignal = 1750;
	if(pwmSignal < 1250) pwmSignal = 1250;

	return pwmSignal; 
}

int pid_Motor_Output(int desired_speed) // desired speed in m/s
{//	ROS_INFO("Pid motor");
	//calculate errors
	float previousErr = err_m;
	
	err_m = desired_speed - currentSpeed;

	long timeNow = currentTimeSpeed.nsec;

	//time between now and last roll message we got
	double dTnsec = (timeNow - previousTimeSpeed.nsec);///(10e9f); //in sec
	if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
	double dT = dTnsec/(1e9f);

	if(dT > 0)
		derr_m = (err_m - previousErr)/dT;

	Kierr_m += Ki_m*err_m*dT;

	//old anti wind-up (saturation)
	if(Kierr_m > MAX_IERR_MOTOR) Kierr_m = MAX_IERR_MOTOR;
	if(Kierr_m < -MAX_IERR_MOTOR) Kierr_m = -MAX_IERR_MOTOR;
	
	//PID CONTROLLER
	float controlSignal = Kp_m*err_m + Kierr_m + Kd_m*derr_m; // should be between 0 and 20.6m/s (3900*8.4*0.4*0.24*2*pi/60*62.5*10-3)
	
	int pwmSignal = (int)((controlSignal*500.0f)/20.6f)+1500;
	if(pwmSignal > 2000) pwmSignal = 2000;
	if(pwmSignal < 1500) pwmSignal = 1500;

	return pwmSignal; 
}

void read_Imu(sensor_msgs::Imu imu_msg)
{//	ROS_INFO("IMU");
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;
	//ROS_INFO("time prev %d time now %d",previousTime.nsec, currentTime.nsec );
	//current roll angle
	currentRoll = imu_msg.orientation.x;
	ROS_INFO("Time %d", the_time);
	//ROS_INFO("current roll %f", currentRoll);
	if(the_time < 15) RollOffset = currentRoll;
	//ROS_INFO("Roll Offset %f", RollOffset);
	currentRoll -= RollOffset;
	ROS_INFO("New Roll %f", currentRoll);
}

int main(int argc, char **argv)
{
//	ROS_INFO("Start");
	int saturation = 2000;
	int freq = 100;
	Kp_m = 0;
	Ki_m = 0;
	Kd_m = 0;

	ROS_INFO("number of argc %d", argc);

	if(argc == 1)
	{
		//case with default params
	}
	else if(argc == 2)
	{
		//case with frequency
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	}
	else if(argc == 3)
	{
		//case with frequency and saturation
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	
		if(atoi(argv[2]) > 2000) saturation = 2000;
		else saturation = atoi(argv[2]);
	}
	else if(argc == 6)
	{
		//case with frequency and saturation
		if(atoi(argv[1]) > 0 )
			freq = atoi(argv[1]);
		else
		{
			ROS_INFO("Frequency must be more than 0");
			return 0;
		}
	
		if(atoi(argv[2]) > 2000) saturation = 2000;
		else saturation = atoi(argv[2]);

		Kp_m = atof(argv[3]);
		Ki_m = atof(argv[4]);
		Kd_m = atof(argv[5]);
		
	}
	else
	{
		ROS_INFO("not enough arguments ! ");
		return 0;
	}

	ROS_INFO("frequency %d, and saturation  : %d", freq, saturation);


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
	ros::Rate loop_rate(freq);

	/****************************/
	/* Initialize the PID Stuff */
	/****************************/
	
	currentRoll = 0;
	currentTime = ros::Time::now();
	previousTime = ros::Time::now();
	Kierr = 0;
	err = 0;
	derr = 0;

	currentSpeed = 0;
	currentTimeSpeed = ros::Time::now();
	previousTimeSpeed = ros::Time::now();
	Kierr_m = 0;
	err_m = 0;
	derr_m = 0;
	
	/*******************************************/
	/* Initialize the RC input, and PWM output */
	/*******************************************/

	RCInput rcin;
	rcin.init();
	PWM servo;
	PWM motor;

	if (!motor.init(MOTOR_PWM_OUT)) {
		fprintf(stderr, "Motor Output Enable not set. Are you root?\n");
		return 0;
    	}

	if (!servo.init(SERVO_PWM_OUT)) {
		fprintf(stderr, "Servo Output Enable not set. Are you root?\n");
		return 0;
    	}
	//ROS_INFO("l");
	motor.enable(MOTOR_PWM_OUT);
	servo.enable(SERVO_PWM_OUT);

	motor.set_period(MOTOR_PWM_OUT, 50); //frequency 50Hz
	servo.set_period(SERVO_PWM_OUT, 50); 

	int motor_input = 0;
	int servo_input = 0;

	sensor_msgs::Temperature rem_msg;
	sensor_msgs::Temperature ctrl_msg;

	float desired_roll = 0;
	float desired_speed = 0;

	//speed in m/s
	float speed = 0;
	float speed_filt = 0;
	int dtf = 0;// dtf read from arduino. dtf = dt*4 in msec
	float R = 0.0625f; //Rear Wheel Radius
	//ROS_INFO("Init");
	RollOffset = 0;
	int initTime = ros::Time::now().sec%10000;

	// LED Init ( this is to know if 8 shape is active or not )
	//RGBled led;
	//if(!led.initialize()) return EXIT_FAILURE;
	bool active_8 = false;
	(rcin.read(4)>1500)?(active_8 = true):(active_8 = false);
	//(active_8)?(led.setColor(Colors::Green)):(led.setColor(Colors::Red));

	int ref_counter = 0; // counter for reference angle
	int ref_size = (sizeof(lookup_roll))/sizeof(lookup_roll[0]); // length of array

	while (ros::ok())
	{
		// update the state of the LED and check for change in AUX button
		if( active_8)
		{
			if(rcin.read(4) <= 1500)
			{
				active_8 = !active_8;
	//			led.setColor(Colors::Red);
			}
		}
		else
		{
			if(rcin.read(4) > 1500)
			{
				active_8 = !active_8;
				ref_counter = 0;
	//			led.setColor(Colors::Green);
			}
		}		

		ref_counter ++;
		if(ref_counter >= ref_size) ref_counter -= ref_size;
		if(active_8) desired_roll = lookup_roll[ref_counter];
		else
		{
			//read desired roll angle with remote ( 1250 to 1750 ) to range of -30 to 30 deg
			desired_roll = -((float)rcin.read(2)-1500.0f)*MAX_ROLL_ANGLE/250.0f;
			//ROS_INFO("rcin usec = %d    ---   desired roll = %f", rcin.read(2), desired_roll);
		}
		//Get Desired PWM Speed using Throttle saturation
		int desired_pwm = 0;
		if(rcin.read(3) >= saturation)
			desired_pwm = saturation;
		else
			desired_pwm = rcin.read(3);
	//	ROS_INFO("mid");
		//get derired speed in m/s using desired pwm
		desired_speed = 20.6f*((float)desired_pwm-1500)/(500.0f);
		if(desired_speed < 0) desired_speed = 0.0f;

		//Read current Speed in m/s
		dtf = rcin.read(5)-1000;
		speed = 4.0f*PI*R*1000.0f/((float)dtf);
		if(speed < 0 || dtf < 40) speed = 0;
		
		// low pass filtering of the speed with tau = 0.1
		float alpha = (1.0f/freq)/((1.0f/freq)+0.1f);
		speed_filt = alpha*speed + (1.0f-alpha)*speed_filt;

		//update time for speed control
		currentSpeed = speed_filt;
		previousTimeSpeed = currentTimeSpeed;
		currentTimeSpeed = ros::Time::now();

		//calculate output to motor from pid controller
		motor_input = pid_Motor_Output(desired_speed);
		if(desired_pwm < 1500)
			motor_input = desired_pwm;
		//calculate output to servo from pid controller
		servo_input = pid_Servo_Output(desired_roll);
		
		//write readings on pwm output
		motor.set_duty_cycle(MOTOR_PWM_OUT, ((float)motor_input)/1000.0f); 
		servo.set_duty_cycle(SERVO_PWM_OUT, ((float)servo_input)/1000.0f);

		
		//ROS_INFO("Ros Time");
		the_time = ros::Time::now().sec%10000-initTime;
		//ROS_INFO("%d", time);
		//Calibration of Roll measurement !
		


		//save values into msg container a
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = desired_speed;//motor_input;
		rem_msg.variance = desired_roll;//servo_input;

		//save values into msg container for the control readings
		ctrl_msg.header.stamp = ros::Time::now();
		ctrl_msg.temperature = currentSpeed;
		ctrl_msg.variance = currentRoll;//desired_roll;//here it's supposed to be the desired roll

		ROS_INFO("DESIRED SPEED : %2.2f     CURRENT SPEED %2.2f", desired_speed, currentSpeed);
		//debug info
		//printf("[Thrust:%d] - [Steering:%d] - [dtf:%4d] - [Speed:%2.2f]\n", motor_input, servo_input, dtf, speed_filt);
		printf("RcVals > %4d %4d %4d %4d %4d %4d %4d %4d\n",rcin.read(0), rcin.read(1), rcin.read(2), rcin.read(3), rcin.read(4), rcin.read(5), rcin.read(6), rcin.read(7));
		//remote_pub.publish(apub);
		remote_pub.publish(rem_msg);
		control_pub.publish(ctrl_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

