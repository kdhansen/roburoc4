// State Estimator for the RobuROC4.
//
// State estimation node estimates the states of the robot based on the sensor redings, 
// the velocity given to the vehicle, and the model.
// For more information about the controller principle refer to 
// roburoc4_documentation/project_reports/12gr832_robotic_road_striper.pdf
// This is a part of the ASETA project, Aalborg University.
// 
// Copyright 2012 Rune Madsen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as publishSegmented by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/* ROS things */
#include <ros/ros.h>
#include <ros/console.h>

/* Sensor messages */
#include <sensor_msgs/NavSatFix.h>				// Message types for the GPS position output
#include <sensor_msgs/NavSatStatus.h>

#include <geometry_msgs/Twist.h>				// Message type to contain velocities
#include <geometry_msgs/Pose2D.h>				// Message type to contain heading

#include <std_msgs/Empty.h>						// Message containing no data. Just used to poke

/* Estimated states output */
#include <roburoc4/States.h>	// Message type to contain the states of the system

/* misc */
#include <std_msgs/String.h>					// Messages to print string to a topic

/* Included libraries */
#include <armadillo>							// Matrix manipulation lib
#include <GeographicLib/UTMUPS.hpp>				// Conversion between lat/long and UTM

/* Misc defines */
#define ANTENNA_THETA_C 0.5637					// The displacement of the antenna	
#define ANTENNA_L 0.64475

#define sign(x) (( x > 0 ) - ( x < 0 ))			// The sign function

using namespace arma;
using namespace std;
using namespace GeographicLib;

/* The state estimation class */
class StateEstimationNode
{
  public:
	StateEstimationNode();
	~StateEstimationNode();
	
	/* Loop running the filter */
	void loop();

  private:
	/* The ros node handle */
	ros::NodeHandle nh_;

	/* Filter Settings */
	int filterRate_;		// Filter rate in Hz
	
	/* Callback functions for sensor inputs */
	void gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& pos );
	void gpsVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel );
	void gpsHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head );
	
	void gyroVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel );
	
	void magnetometerHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head );
	
	void roc4VelocityCallback(const geometry_msgs::Twist::ConstPtr& vel );
	
	void setVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVel );
	
	void resetFilterCallback(const std_msgs::Empty::ConstPtr& dummy );
	
	/* The publisher and subscriper */
	ros::Subscriber gpsPosSub_;				// GPS Position
	ros::Subscriber gpsVelSub_;				// GPS Linear Velocity
	ros::Subscriber gpsHeadSub_;			// GPS Heading
	ros::Subscriber gyroVelSub_;			// Gyro Angular Velocity
	ros::Subscriber magHeadSub_;			// Magnetometer Heading
	ros::Subscriber roc4VelSub_;			// RobuROC4 Linear and Angular Velocity
	
	ros::Subscriber setVelSubscriber_;		// Velocity command send form a controlle to the RobuROC4
	
	ros::Subscriber resetFilterCmdSub_;		// Command to reset the filter!
	
	ros::Publisher statePub_;  				// The estimated states of the robot
	ros::Publisher stringPub_;				// A string!
		
	/* Matrices and other good stuff for the filter */
	float Ts;								// The sampling time
	mat A,Al,B,C,D;							// System matrices
	mat x, xOld;							// State Matrices
	mat y, yMeas;							// Measurement ( and model output )
	mat Q, R, P;							// Covariance Model, Measurement, Current estimate
	mat u;									// System input
	mat K;									// Kalman Gains
	
	double invalidData;						// Covariance for invalid data
	double R1,R2,R3,R4,R5,R6,R7,R8;			// Sensor covariances for perfect valid data
	double Q1,Q2,Q3,Q4,Q5,Q6,Q7;			// Model covariances for perfect valid model prediction
};


StateEstimationNode::StateEstimationNode():
  filterRate_(20)
{
	/* Get the parameters from the parameter server */
	nh_.param("/state_estimation_node/filterRate", filterRate_, filterRate_);

	/* Generate the subscribers */
	gpsPosSub_ = nh_.subscribe<sensor_msgs::NavSatFix>("GPS/position", 1, &StateEstimationNode::gpsPositionCallback, this);
	gpsVelSub_ = nh_.subscribe<geometry_msgs::Twist>("GPS/velocity", 1, &StateEstimationNode::gpsVelocityCallback, this);
	gpsHeadSub_ = nh_.subscribe<geometry_msgs::Pose2D>("GPS/orientation", 1, &StateEstimationNode::gpsHeadingCallback, this);
	
	gyroVelSub_ = nh_.subscribe<geometry_msgs::Twist>("Gyro/velocity", 1, &StateEstimationNode::gyroVelocityCallback, this);
	
	magHeadSub_ = nh_.subscribe<geometry_msgs::Pose2D>("Magnetometer/orientation", 1, &StateEstimationNode::magnetometerHeadingCallback, this);
	
	roc4VelSub_ = nh_.subscribe<geometry_msgs::Twist>("roburoc4_driver/cur_vel", 1, &StateEstimationNode::roc4VelocityCallback, this);
	
	setVelSubscriber_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1 , &StateEstimationNode::setVelCallback, this);
	
	resetFilterCmdSub_ = nh_.subscribe<std_msgs::Empty>("roburoc4_state_estimator/reset_filter", 1, &StateEstimationNode::resetFilterCallback, this);
	
	/* Publisher */
	statePub_ = nh_.advertise<roburoc4::States>("roburoc4_state_estimator/states", 10);
	stringPub_ =  nh_.advertise<std_msgs::String>("roburoc4_printString", 10);
		
	/** Setup the filter matrices **/
	Ts = 1.0/(float)filterRate_ ;	// Get the sampling time	

	double theta = 0;	// dummy theta to be able to show the structure of the matrices 
	x.zeros(7);			// dummy input to be able to show the structure of the matrices 
	

	/* Define the A-matrix (update theta on every sample) */
	A 	<<	1	<<	0	<<	0	<<	0			<<	0			<<	cos(theta)*Ts	<<	0 				<<	endr // [x]
        <<	0	<<	1	<<	0	<<	0			<<	0			<<	sin(theta)*Ts	<<	0 				<<	endr // [y]
        <<	0	<<	0	<<	1	<<	Ts			<<	0			<<	0				<<	0 				<<	endr // [theta]
        <<	0	<<	0	<<	0	<<	1			<<	Ts			<<	0				<<	0 				<<	endr // [q1]
        <<	0	<<	0	<<	0	<<	-5.8342*Ts	<<	1-3.7323*Ts	<<	0				<<	0 				<<	endr // [q2]
        <<	0	<<	0	<<	0	<<	0			<<	0			<<	1				<<	Ts 				<<	endr // [q3]
        <<	0	<<	0	<<	0 	<<	0			<<	0			<<	-6.0384*Ts		<<	1-3.8630*Ts 	<<	endr; //[q4]
        
    /* Define the Al-matrix (Linearised A-matrix) (update theta on every sample)  */
	Al 	<<	1	<<	0	<<	-sin(theta)*x(5)*Ts		<<	0			<<	0			<<	cos(theta)*Ts	<<	0 				<<	endr // [x]
        <<	0	<<	1	<<	cos(theta)*x(5)*Ts		<<	0			<<	0			<<	sin(theta)*Ts	<<	0 				<<	endr // [y]
        <<	0	<<	0	<<	1						<<	Ts			<<	0			<<	0				<<	0 				<<	endr // [theta]
        <<	0	<<	0	<<	0						<<	1			<<	Ts			<<	0				<<	0 				<<	endr // [q1]
        <<	0	<<	0	<<	0						<<	-5.8342*Ts	<<	1-3.7323*Ts	<<	0				<<	0 				<<	endr // [q2]
        <<	0	<<	0	<<	0						<<	0			<<	0			<<	1				<<	Ts 				<<	endr // [q3]
        <<	0	<<	0	<<	0 						<<	0			<<	0			<<	-6.0384*Ts		<<	1-3.8630*Ts 	<<	endr;// [q4]        
		
	/* Define the B-matrix */
	B	<<	0			<<  0			<< endr
		<<	0   		<<	0			<< endr
        << 	0   		<< 	0			<< endr
        << 	0   		<<	0.7664*Ts	<< endr
        << 	0     		<<	2.9738*Ts	<< endr
		<<	0.7932*Ts 	<<  0			<< endr
		<<	2.9743*Ts 	<<  0			<< endr;
		
	/* Define the C-matrix */
	C	<<	1	<<	0	<<	0	<<	0	<<	0	<<	0	<<	0	<< endr
		<<	0	<<	1	<<	0	<<	0	<<	0	<<	0	<<	0	<< endr
		<<	0	<<	0	<<	1	<<	0	<<	0	<<	0	<<	0	<< endr
		<<	0	<<	0	<<	1	<<	0	<<	0	<<	0	<<	0	<< endr
		<<	0	<<	0	<<	0	<<	1	<<	0	<<	0	<<	0	<< endr
		<<	0	<<	0	<<	0	<<	1	<<	0	<<	0	<<	0	<< endr
		<<	0	<<	0	<<	0	<<	0	<<	0	<<	1	<<	0	<< endr
		<<	0	<<	0	<<	0	<<	0	<<	0	<<	1	<<	0	<< endr;
		
	/* The D-Matrix only consists of  0'es therefore it is simply not used! */
	
	/* Define Covariance for very imprecise data */
	invalidData = 1000000000;
	
	/* Define Covariances for the valid sensor data */
	R1 = 0.003253; // [gps_x] [m]
	R2 = 0.003253; // [gps_y] [m]
	R3 = 0.002721; // [gps_theta] [rad]
	R4 = 0.0564;   // [magnetometer_theta] [rad] // NOTE: Multiplied by a factor of 100 to decrease influence since it drifts!
	R5 = 0.000025; // [gyro_theta^(1)] 
	R6 = 0.000053; // [roc4_theta^(1)]
	R7 = 0.003618; // [gps_v]
	R8 = 0.000041; // [roc4_v]
	
	/* Define Covariances for the valid model (Handtuned to optain the desired shape of the filter)*/
	Q1 = 0.00001; 	// [x]
	Q2 = 0.00001; 	// [y]
	Q3 = 0.00001; 	// [theta]
	Q4 = 0.00001;	// [theta^(1)]
	Q5 = 0.00001;	// [q1]
	Q6 = 0.000004; 	// [v]
	Q7 = 0.000001; 	// [q2]
	
	
	/* Generate the matrices with the correct dimensions and correct initial values */
	x.zeros(7);		// State vector
	xOld.zeros(7);
	
	y.zeros(8);		// Measurement vector
	yMeas.zeros(8);
	
	u.zeros(2);		// Model input vector
	
	R.zeros(8,8);	// Measurement Covariances
	
	Q.zeros(7,7);	// Model Covariances
	Q(0,0) = Q1;
	Q(1,1) = Q2;
	Q(2,2) = Q3;
	Q(3,3) = Q4;
	Q(4,4) = Q5;
	Q(5,5) = Q6;
	Q(6,6) = Q7;
	
	
	P = invalidData * eye<mat>(7,7);	// Initial state covariance

	ROS_INFO_STREAM("RobuROC4_state_estimator: State estimator is up and running...");
		
}

StateEstimationNode::~StateEstimationNode(){
			
}

/* The main loop for the state estimation */
void StateEstimationNode::loop(){
	
	ros::Rate r(filterRate_);
	
	while(ros::ok()){
	
		/* Reset the Covariance matrices to indicate "no measurements" */
		R = invalidData * eye<mat>(8,8);	// Invalid measurement data
	
		/* Spin (once) to capture new data including an update of the measurement covariance matrix */
		ros::spinOnce();
		
		/********** Do the filtering ****************/
		xOld = x;				// Save the old state vector
		double theta = x(2);	// Extract the angle
		
		// Update both the nonlinear and the linearized system matrix
		A(0,5) = cos(theta)*Ts;
		A(1,5) = sin(theta)*Ts;
		
		Al(0,2) = -sin(theta)*x(5)*Ts;	Al(0,5) = cos(theta)*Ts;
		Al(1,2) = cos(theta)*x(5)*Ts;	Al(1,5) = sin(theta)*Ts;
		
		/* Predict the new state and output */
		x = A*xOld + B*u;
		y = C*x;
		
		/* Propagate the covariance using the linearized system evaluated in the current states */
		P = Al*P*Al.t() + Q;
		
		/* Find the optimal gain (kalman gain) */
		K = P*C.t()*pinv(C*P*C.t()+R);								
				
		/* Modify heading measurements to be within +/- pi of the model output */
		if (abs(yMeas(2)-y(2)) > M_PI) {							// GPS Heading
			yMeas(2) = yMeas(2) - sign(yMeas(2)-y(2)) * 2 * M_PI;
		}
		if (abs(yMeas(3)-y(3)) > M_PI) {							// Magnetometer Heading
			yMeas(3) = yMeas(3) - sign(yMeas(3)-y(3)) * 2 * M_PI;
		}
		
		/* Update the states */
		x = x + K*(yMeas-y);
		
		// make sure that the angle is within [0 2pi];
		x(2) = fmod( x(2) + 2 * M_PI, 2 * M_PI);
		
		/* Update the state covariances */
		P = ( eye<mat>(7,7) - K * C ) * P;
		
		/**** Filter completed ****/
		
		/* Publish the current states */
		roburoc4::States states;
		
		states.x = x(0);
		states.y = x(1);
		states.theta = x(2);
		states.omega = x(3);
		states.q1 = x(4);
		states.v = x(5);
		states.q2 = x(6);
		
		statePub_.publish(states);
				
		/* Sleep to obtain the desired filter rate */
		r.sleep();	
	}
}

/** Callbacks to capture data from the sensors **/
void StateEstimationNode::gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& pos ){
	
	/* Extract the data from the topic */
	double latitude = pos->latitude;
	double longitude = pos->longitude;	
	
	/* If the data isnt 0 do something */
	if ( latitude != 0 && longitude != 0 ) {
	
		int zone;
		bool northp;
		double utmx,utmy;
	
		/* Convert the data */
		UTMUPS::Forward(latitude, longitude, zone, northp, utmx, utmy);
		
		/* Update the measurement vector */
		yMeas(0) = utmx;
		yMeas(1) = utmy;
		
		/* Move the GPS antenna to the centre of the vehicle by use of the estimated heading */
		yMeas(0) += cos( x(2) - ANTENNA_THETA_C ) * ANTENNA_L;
		yMeas(1) += sin( x(2) - ANTENNA_THETA_C ) * ANTENNA_L;
		
		/* Update the measurement covariance matrix */
		R(0,0) = R1;
		R(1,1) = R2;
		
	}
}

void StateEstimationNode::gpsVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel ){
	
	// Set the measurement to the velocity
	yMeas(6) = vel->linear.x;
	
	// Make the velocity data valid
	R(6,6) = R7;
	
}

void StateEstimationNode::gpsHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head ){

	double theta = head->theta;

	// If the current velocity is negative, add pi to the angle
	if (x(5) < 0) {
		theta = fmod( theta + M_PI, 2 * M_PI);
	}
	// Set the measurement
	yMeas(2) = theta;
		
	// If the linear velocity is high enough, and the angular velocity is low enough, make the heading data valid
	if ( abs(x(3)) < 0.3 ) {
		if ( abs(x(5)) > 2.0 ) {
			R(2,2) = R3;					// Ideal condition
		} else if ( abs(x(5)) > 0.25 ) {
			R(2,2) = 0.032059;				// OK condition
		}
	} 
}
	
void StateEstimationNode::gyroVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel ){
	
	/* Extract the measurement from the topic */
	yMeas(4) = vel->angular.z;
	
	/* If the velocity is below 99 deg/s make the measurement valid */
	if ( yMeas(4) < 99 * M_PI / 180 )
		R(4,4) = R5;

}
	
void StateEstimationNode::magnetometerHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head ){
	
	/* Extract the measurement from the topic */
	yMeas(3) = head->theta;
	
	/* Make the measurement valid */
	R(3,3) = R4;
	
}
	
void StateEstimationNode::roc4VelocityCallback(const geometry_msgs::Twist::ConstPtr& vel ){
	
	/* Extract the measurements from the topic */
	yMeas(5) = vel->angular.z;
	yMeas(7) = vel->linear.x;
	
	/* Make the data valid */
	R(5,5) = R6;
	R(7,7) = R8;
	
}

void StateEstimationNode::setVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVel ) {
	/* Set the model input from the set vel message */
	u(0) = cmdVel->linear.x;
	u(1) = cmdVel->angular.z;
}

void StateEstimationNode::resetFilterCallback(const std_msgs::Empty::ConstPtr& dummy ) {
	/* Reset the filter by setting the states to zero and the variance of the states very high */
	
	x.zeros(7);							// Reset states
	P = invalidData * eye<mat>(7,7);	// Reset covariance
	
	y.zeros(8);							// Reset measurement vector
	yMeas.zeros(8);
	
	u.zeros(2);							// Reset model input vector
	
	ROS_DEBUG_STREAM("RobuROC4 state estimator: Filter reset!");
	
	
	/* Publish filter reset message to a topic */
	std_msgs::String aString;
	aString.data = "Kalman Filter is reset!";
	stringPub_.publish(aString);
	
}

/* The main function starts the estimation loop */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "roburoc4_state_estimator");
	StateEstimationNode stateEstimationNode;

	stateEstimationNode.loop();

	return 1;
}
