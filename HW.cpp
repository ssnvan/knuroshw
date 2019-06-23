/*
2016114418 bae keun ryeong
HW03
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <float.h>
#influce <stdlib.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
//boost::mutex mutex;
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
boost::mutex mutex[2];
float pre_dAngleTurned;

template<typename T>
inline bool isnan(T value)
{
	return value != value;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}

scanMsgCallback(const sensor_msgs::LaserScan& msg) {
	mutex[1].lock(); {
		g_scan = msg;
	}
	mutex[1].unlock();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan)
{
	int nRangeSize = (int)lrfScan.ranges.size();
	int range_count = 0;
	double range_avg=0;
	for (int i = 0; i<nRangeSize; i++) {
		double dRange = lrfScan.ranges[i];
		if (i > 45 && i < 315) continue;
		if (!isnan(dRange) && dRange >0 && dRange <3) {
			range_count++;
			range_avg += dRange;
			
		}
	}
	if (range_count > 0)
		return (range_avg / range_count);
	else
		return 10;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// odom으로부터 현재의 변환행렬 정보를 리턴!
tf::Transform
getCurrentTransformation(void)
{
	// transformation 버퍼
	tf::Transform transformation;

	// odom 버퍼
	nav_msgs::Odometry odom;

	// copy a global '/odom' message with the mutex
	mutex[0].lock(); {
		odom = g_odom;
	} mutex[0].unlock();

	// 위치 저장
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	// 회전 저장
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	// 리턴
	return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 로봇이 멈춰있는 상태(처음 상태)의 위치를 저장!
tf::Transform
getInitialTransformation(void)
{
	// tf 변환행렬
	tf::Transform transformation;

	// 처음위치에 대한 odometry 메시지 받기
	ros::Rate loopRate(1000.0);

	while (ros::ok()) {
		// 일단 callback 메시지를 받고!
		ros::spinOnce();

		// get current transformationreturn;
		transformation = getCurrentTransformation();

		// 메시지를 받았으면 break!
		if (transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
			break;
		}
		else {
			loopRate.sleep();
		}
	}

	// 리턴
	return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 회전실행
bool
doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	//the command will be to turn at 'rotationSpeed' rad/s
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;

	if (dRotation < 0.) {
		baseCmd.angular.z = -dRotationSpeed;
	}
	else {
		baseCmd.angular.z = dRotationSpeed;
	}

	// 이동하면서 현재위치에 대한 odometry 메시지 받기
	bool bDone = false;
	ros::Rate loopRate(1000.0);



	while (ros::ok() && !bDone) {
		// 일단 callback 메시지를 받고!

		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();

		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation;
		tf::Quaternion rotationQuat = relativeTransformation.getRotation();



		double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]), (1 - (2 * (rotationQuat[2] * rotationQuat[2]))));

		// 종료조건 체크

		if (fabs(dAngleTurned) > fabs(dRotation) || (dRotation == 0))
		{
			bDone = true;
			break;
		}
		else {
		//	pre_dAngleTurned = dAngleTurned;
			//send the drive command
			pubTeleop.publish(baseCmd);

			// sleep!
			loopRate.sleep();
		}
	}

	// Initialization
	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 이동
bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
	//the command will be to go forward at 'translationSpeed' m/s
	geometry_msgs::Twist baseCmd;
/*
	if (dTranslation < 0) {
		baseCmd.linear.x = -dTranslationSpeed;
	}
	else {
		baseCmd.linear.x = dTranslationSpeed;
	}
	*/
	baseCmd.linear.x = dTranslationSpeed;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;

	// 이동하면서 현재위치에 대한 odometry 메시지 받기
	bool bDone = false;
	ros::Rate loopRate(1000.0);

	while (ros::ok() && !bDone) {
		// 일단 callback 메시지를 받고!
		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();

		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation;
		double dDistMoved = relativeTransformation.getOrigin().length();

		// 종료조건 체크

		if (fabs(dDistMoved) >= fabs(dTranslation)) {
			bDone = true;
			break;
		}
		else {
			//send the drive command
			pubTeleop.publish(baseCmd);

			// sleep!
			loopRate.sleep();
		}
	}

	//  초기화

	baseCmd.linear.x = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
	// ROS 초기화
	ros::init(argc, argv, "turtle_position_move");

	// Ros initialization
	ros::NodeHandle nhp, nhs;

	// Decleation of subscriber
	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
	ros::Subscriber sub_scan = nhs.subscribe("/scan", 100, &scanMsgCallback);
	// Create a publisher object
	ros::Publisher pubTeleop = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	geometry_msgs::Twist baseCmd;
	nav_msgs::Odometry curodom;
	sensor_msgs::LaserScan curscan;
	baseCmd.linear.x = 0.4;
	baseCmd.linear.y = 0.0;
	baseCmd.linear.z = 0.0;
	baseCmd.angular.z = 0.0;
	//float _dRatation = (float)((int)dRotation % 360);
	//double dTranslation = atof(argv[2]);


	// 로봇이 멈춰있는 상태(처음 상태)의 변환행렬 가져오기
	//tf::Transform initialTransformation = getInitialTransformation();

	// 회전

	//doRotation(pub, initialTransformation, toRadian(dRotation), 0.75);
	//doRotation(pub, initialTransformation, 3.14*2, 0.75);

	// 이동
	//doTranslation(pub, initialTransformation, dTranslation, 0.25);

	tf::Transform initialTransformation;
	tf::Transform curTrans;
	double avg;
	bool obstacle=false;
	while (ros::ok()) {
		ros::spinOnce;
		initialTransformation = getInitialTransformation();
		curTrans = getCurrentTransformation();
		mutex[0].lock(); { curodom = g_odom; }mutex[0].unlock();
		printf("Robot :: %.3lf, %.3lf *****************\n", curodom.pose.pose.position.x, curodom.pose.pose.position.y);
		mutex[1].lock(); { curscan = g_scan; }mutex[1].unlock();
		avg = convertScan2XYZs(scan);
		printf("Obstacnle dist :: %.3lf ******************\n", avg);
		if (avg > 0 && avg < 0.8) {
			baseCmd.linear.x = 0;
			baseCmd.linear.y = 0;
			baseCmd.linear.z = 0;
			obstacle = true;
		}
		pubTeleop.publish(baseCmd);
		double deg = 0;
		if (obstacle) {
			printf("Detected Obstacle.......\n\n");
			srand(time(NULL));
			if (rand() % 2 == 1) deg = toRadian(80);
			else deg = -toRadian(80);

			curTrans = getCurrentTransformation();
			doRotation(pubTeleop, currentTransformation, deg, 0.3);
			obstacle = false;
			baseCmd.linear.x = 0.4;
			baseCmd.linear.y = 0.0;
			baseCmd.linear.z = 0.0;
			baseCmd.linear.z = 0.0;
			printf("Finish Rotating..........\n\n");
		}
	}
	int nKey = waitKey(30) % 255;
	if (nKey == 27) {
		break;
	}
	return 0;
}