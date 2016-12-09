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

//parameters of the controllers, are also KP, Ki, Kd
float K1 = -4.5463;	//-64.1997/12; //-64.1997;
float K2 = -1.0975;	//-1.908;
float K3 = 0;
float u = 0;		// the desired angle of the pilot

float currentRoll;
ros::Time currentTime;
ros::Time previousTime;
float rollOffset;			//calculated by the imu

float currentRollSpeed;
float speedOffset = -2.25;		//empirical estimation, for LQR controller
float trueSpeed;

float correction;

//Roll Errors 1
float err;
float derr;
float Kierr;
float MAX_IERR = 50;

//PID parameters
float Kp;
float Kd;
float Ki;

int the_time = 0;
double dTnsec = 0;
int freq = 50;

int Pilot_angle(int desired_roll) //in degrees
{
	//calculate errors
	float previousErr = err;
	err = desired_roll - currentRoll;		//should be other way roud but it is correct because the imu gives us positif roll in anticlockwise

	long timeNow = currentTime.nsec;

	//time between now and last roll message we got
	dTnsec = (timeNow - previousTime.nsec); // in nanoseconds
	if(dTnsec < 0) dTnsec += 1e9; // watch out cause its in ns so if it goes beyond 1 sec ...
	double dT = dTnsec/(1e9f);

	if(dT > 0)
		derr = (err - previousErr)/0.02;
	
	Kierr += Ki*err*0.02;

	//anti wind-up (saturation)
	if(Kierr > MAX_IERR) Kierr = MAX_IERR;
	if(Kierr < -MAX_IERR) Kierr = -MAX_IERR;

	//PID CONTROLLER
	float controlSignal = Kp*err + Kierr +  Kd*derr;

	return controlSignal;
}

void read_Imu(sensor_msgs::Imu imu_msg)
{
	//save the time of the aquisition
	previousTime = currentTime;
	currentTime = imu_msg.header.stamp;

	//current roll angle, positif value in clockwise
	currentRoll = imu_msg.orientation.x;		
	currentRollSpeed = imu_msg.angular_velocity.x;
	
	//keep calibration after 15 seconds
	if(the_time < 15) 
	{
		rollOffset = currentRoll;
		ROS_INFO("Hold still for %d secondes: calibration %f",the_time, rollOffset);
	}
	currentRoll -= rollOffset;
	
}

int main(int argc, char **argv)
{	
	double dT_info;
	double max = 70;
	
	/*******************************************/
	/* Definie parameter */
	/*******************************************/
	if((atof(argv[1])) != 0)		//attention, must be atof (and not atoi) because K1 et K2 are decimal numbers
	{
		if((atof(argv[1])) > -max/7  && (atof(argv[1])) < max/7 )
		K1 = atof(argv[1]);
		
		if((atof(argv[2])) > -max  && (atof(argv[2])) < max )
		K2 = atof(argv[2]);
		
		if((atof(argv[3])) > -max  && (atof(argv[3])) < max )
		K3 = atof(argv[3]);
	}
	
	Kp = K1;
	Ki = K2;
	Kd = K3;
	
	ROS_INFO("Beginning with stabilisation with amplitudes: %f, %f and %f", K1, K2, K3);
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
		/*******************************************/
		/* LQR controller */
		/*******************************************/
	  	// indication: degre to amplitude for pilot servo ; 90° = 900 microseconde --> 1° = 10 microsecondes
		trueSpeed = currentRollSpeed - speedOffset;		//manual offset chosen empirically
		u = K1*currentRoll + K2*trueSpeed;			//degree, positif values of imu in clockwise, currentRoll was already amputed by offset
		correction = u*10;					//deg->amplitude pwm in ms
	
		/*******************************************/
		/* PID Controller */
		/*******************************************/
		u = Pilot_angle(0);
		correction = u*10;					//deg->amplitude pwm in ms
		if(correction > 300) Kierr = 300;
		if(correction < -300) Kierr = -300;
		/*******************************************/
		/* Control */
		/*******************************************/
		if(the_time > 15) pilot_input = PILOT_TRIM + correction;	//to avoid moving during calibartion
		
		if( pilot_input < 600)		//limited to min pwm signal, to avoid problems
		pilot_input = 600;
		if( pilot_input > 2400)		//limited to max pwm signal, to avoid problems
		pilot_input = 2400;

		//write readings on pwm output in miliseconds
		pilot.set_duty_cycle(PILOT_PWM_OUT, ((float)pilot_input)/1000.0f);
		
		//save values into msg container for the remote readings
		rem_msg.header.stamp = ros::Time::now();
		rem_msg.temperature = 0; // motor_input;
		rem_msg.variance = pilot_input;
		
		i++;
		if(i == 25)		//To get insgiht on the code and what is happening
		{
		//ROS_INFO("Pilot: %f, Roll: %f, RollOffset: %f\n u: %f, correction %f, dTnsec %d", pilot_input, currentRoll, rollOffset, u, correction, dTnsec/1e9);
		dT_info = dTnsec/(1e9f);
		double Kperr = Kp * err;
		double Kderr = Kd * derr;
		ROS_INFO(" u: %f, dT %f\n Error %f, Kp*err %f, Ierror %f and Derror %f ", u, dT_info, err, Kperr, Kierr, Kderr);
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

		ros::spinOnce();			//Call this function to allow ROS to process incoming messages 

		loop_rate.sleep();			//Sleep for the rest of the cycle, to enforce the loop rate
	}	

return 0;

}
