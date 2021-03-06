#include "MPU9250.h"
#include "LSM9DS1.h"
#include "Util.h"
#include "AHRS.hpp"
#include <unistd.h>
#include <sys/time.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <sstream>

#define G_SI 9.80665
#define PI   3.14159

// Objects

InertialSensor *imu;
AHRS    ahrs;   // Mahony AHRS

// Sensor data

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// Orientation data

float roll, pitch, yaw;

// Timing data

float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

//============================= Initial setup =================================

void imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu->initialize();

    //-------------------------------------------------------------------------

	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
		imu->update();
    imu->read_gyroscope(&gy, &gx, &gz);

    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= 180 / PI;

		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
}

//============================== Main loop ====================================

void imuLoop()
{
    //----------------------- Calculate delta time ----------------------------

	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

    // Accel + gyro.
	/*
    imu->update();
    imu->read_accelerometer(&ax, &ay, &az);
    imu->read_gyroscope(&gx, &gy, &gz);

    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= 180 / PI;

    ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);
    	*/
    // Accel + gyro + mag.
    // Soft and hard iron calibration required for proper function.
    
    imu->update();
    imu->read_accelerometer(&ay, &ax, &az);
    imu->read_gyroscope(&gy, &gx, &gz);
    imu->read_magnetometer(&my, &mx, &mz);

    ax /= -G_SI;
    ay /= -G_SI;
    az /= G_SI;
    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= -180 / PI;

    ahrs.update(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, my, mx, -mz, dt);
    

    //------------------------ Read Euler angles ------------------------------

    //ahrs.getEuler(&pitch, &roll, &yaw);//roll and pitch inverted 
    ahrs.getEuler(&roll, &pitch, &yaw);

    //------------------- Discard the time of the first cycle -----------------

    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;
}

void init_imu_msg(sensor_msgs::Imu* imu_msg)
{
	//time stamp
	imu_msg->header.stamp = ros::Time::now();
	
	imu_msg->orientation.x = 0.0f;
	imu_msg->orientation.y = 0.0f;
	imu_msg->orientation.z = 0.0f;
	imu_msg->orientation.w = 0.0f;

	imu_msg->angular_velocity.x = 0.0f;
	imu_msg->angular_velocity.y = 0.0f;
	imu_msg->angular_velocity.z = 0.0f;

	imu_msg->linear_acceleration.x = 0.0f;
	imu_msg->linear_acceleration.y = 0.0f;
	imu_msg->linear_acceleration.z = 0.0f;

	//Create a cov matrix of [1 0 0; 0 1 0; 0 0 1]
	for(int i = 0; i < 9; i++)
	{
		imu_msg->orientation_covariance[i] = !(i%4);
		imu_msg->angular_velocity_covariance[i] = !(i%4);
		imu_msg->linear_acceleration_covariance[i] = !(i%4);
	}
}

void init_mf_msg(sensor_msgs::MagneticField* mf_msg)
{
	//time stamp
	mf_msg->header.stamp = ros::Time::now();
	
	mf_msg->magnetic_field.x = 0.0f;
	mf_msg->magnetic_field.y = 0.0f;
	mf_msg->magnetic_field.z = 0.0f;

	//Create a cov matrix of [1 0 0; 0 1 0; 0 0 1]
	for(int i = 0; i < 9; i++)
	{
		mf_msg->magnetic_field_covariance[i] = !(i%4);
	}
}

void update_imu_msg(sensor_msgs::Imu* imu_msg, InertialSensor* imu)
{
	//time stamp
	imu_msg->header.stamp = ros::Time::now();
	
	//float ax, ay, az, gx, gy, gz;

	//imu->update();

	//imu->read_accelerometer(&ax, &ay, &az);
        //imu->read_gyroscope(&gx, &gy, &gz);

	imu_msg->orientation.x = roll;
	imu_msg->orientation.y = pitch;
	imu_msg->orientation.z = yaw;
	imu_msg->orientation.w = dt;

	imu_msg->angular_velocity.x = gx;
	imu_msg->angular_velocity.y = gy;
	imu_msg->angular_velocity.z = gz;

	imu_msg->linear_acceleration.x = ax;
	imu_msg->linear_acceleration.y = ay;
	imu_msg->linear_acceleration.z = az;

	ROS_INFO("Accelerometer : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", ax, ay, az);
	ROS_INFO("Gyroscope : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", gx, gy, gz);
	ROS_INFO("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, yaw, dt, int(1/dt));
}

void update_mf_msg(sensor_msgs::MagneticField* mf_msg, InertialSensor* imu)
{
	//time stamp
	mf_msg->header.stamp = ros::Time::now();
	
	//float mx, my, mz;

	//imu->update();

        //imu->read_magnetometer(&mx, &my, &mz);

	mf_msg->magnetic_field.x = mx;
	mf_msg->magnetic_field.y = my;
	mf_msg->magnetic_field.z = mz;

	ROS_INFO("Magnetic Field : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", mx, my, mz);
}

int main(int argc, char **argv)
{
 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "imu_lsm9ds1_handler");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_readings", 1000);
	ros::Publisher mf_pub = n.advertise<sensor_msgs::MagneticField>("mag_readings", 1000);

	//running rate = 30 Hz
	ros::Rate loop_rate(30);

	/*************************/
	/* Initialize the Sensor */
	/*************************/

	printf("Selected: LSM9DS1\n");
	imu = new LSM9DS1();

	/***************/
	/* Test Sensor */
	/***************/
	if (!imu->probe()) 
	{
		printf("Sensor not enabled\n");
		return EXIT_FAILURE;
	}

	imuSetup();

	while (ros::ok())
	{
		imuLoop();		
		
		sensor_msgs::Imu imu_msg;
		sensor_msgs::MagneticField mf_msg;

		init_imu_msg(&imu_msg);
		init_mf_msg(&mf_msg);
		
		update_imu_msg(&imu_msg, imu);
		update_mf_msg(&mf_msg, imu);

		imu_pub.publish(imu_msg);
		mf_pub.publish(mf_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}

