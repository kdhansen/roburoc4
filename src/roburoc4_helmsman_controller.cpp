// RobuROC4 trajectory controller.
//
// Based on work of projectgroup 11gr731. Main idea is to follow a line defined by its two endpoints.
// For more information about the controller principle refer to 
// roburoc4_documentation/project_reports/11gr731_line_tracking_worksheets.pdf and
// roburoc4_documentation/project_reports/11gr731_line_tracking_paper.pdf
// This is a part of the ASETA project, Aalborg University.
// 
// Copyright 2013 Karl Damkjær Hansen
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

/* Messages */
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>	
#include <roburoc4/States.h>
#include <roburoc4/LineSegment.h>

/* Common Controller Defines */
#include <roburoc4_controllers.hpp>

/* Helmsmann Controller Defines */
#define HELMSMAN_MIN_LINEAR_VEL 		-0.5	// Robot min input is -2.6*1.3 m/s
#define HELMSMAN_MAX_LINEAR_VEL 		0.5	// Robot max input is +2.6*1.3 m/s
#define HELMSMAN_MAX_ABS_ANGULAR_VEL 		3.0	// Robot max input is +/- 6.5*1.3 rad/s

#define HELMSMAN_METERS_AIM			2.0	// If far from the goal aim 2 meters ahead
#define HELMSMAN_FACTOR_AIM			2.0	// If close to goal the fraction of the distance to the goal to aim ahead.
							// The threshold between far and close is HELMSMAN_METERS_AIM * HELMSMAN_FACTOR_AIM 
											
#define HELMSMAN_KP_LIN				1.0	// Gains for the controllers linear velocity gain
#define HELMSMAN_KP_ANG				2.0	// Angular velocity gain

#define HELMSMAN_ENDPOINT_THRESHOLD	1.0		// If closer to the endpoint than this value turn off the controller

using namespace std;

class HelmsmanControllerNode {
	
	public:
		HelmsmanControllerNode();
		~HelmsmanControllerNode();
		
		void controlLoop();
		
		
		/* Start and end points */
		double xStart, yStart;
		double xEnd, yEnd;
		
		
	private:
		/* The ros node handle */
		ros::NodeHandle nh_;
		
		/* Publishers */
		ros::Publisher velPub_;
		ros::Publisher stringPub_;				// A string!
		
		/* Subscriber */
		ros::Subscriber stateSub_;
		ros::Subscriber lineSegmentSub_;
		ros::Subscriber controlModeSub_;
		
		/* Callback function for receiving the trajectory and current states */
		void statesCallback(const roburoc4::States::ConstPtr& states);
		void lineSegmentCallback(const roburoc4::LineSegment::ConstPtr& lSeg);
		void controlModeCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg);
		
		/* Function to saturate the velocities such that they are within the limits of the RobuROC4 and publish it */
		void publishVelocity(double linear,double angular);
		
		/* Current states */
		roburoc4::States states;
		
		/* Current line segment */
		roburoc4::LineSegment lineSegment;
			
		/* Control mode off / helmsmann / trajectory */
		unsigned char controlMode;
	
};

HelmsmanControllerNode::HelmsmanControllerNode() {
	
		/* Velocity publisher */
		velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		/* Publish a string to the menu */
		stringPub_ =  nh_.advertise<std_msgs::String>("roburoc4_printString", 10);
		
		/* Subscriber for the states */
		stateSub_ = nh_.subscribe<roburoc4::States>("roburoc4_state_estimator/states", 1 , &HelmsmanControllerNode::statesCallback, this);
		
		/* Subscriber for the LineSegments */
		lineSegmentSub_ = nh_.subscribe<roburoc4::LineSegment>("roburoc4_controllers/lineSegment", 1 , &HelmsmanControllerNode::lineSegmentCallback, this);
		
		/* Subscriber for the controlMode */
		controlModeSub_ = nh_.subscribe<std_msgs::UInt8>("roburoc4_controllers/controlMode", 1 , &HelmsmanControllerNode::controlModeCallback, this);
		
		/* Set the controller to be disabled by default */
		controlMode = CONTROL_MODE_OFF;
		
		/* set the initial start and end pos */
		lineSegment.start.x = 0;
		lineSegment.start.y = 0;
		lineSegment.end.x = 0;
		lineSegment.end.y = 0;
		
}

HelmsmanControllerNode::~HelmsmanControllerNode() {

}

void HelmsmanControllerNode::statesCallback(const roburoc4::States::ConstPtr& newStates){
	
	/* Update the local copy of the current states */
	states.x = newStates->x;
	states.y = newStates->y;
	states.theta = newStates->theta;
	
}

void HelmsmanControllerNode::controlLoop() {

	/* Define control rate */
	ros::Rate r(20);
	
	/* Variables for the calculated velocities */
	double linear, angular;
	
	/* Controller factors */
	double m_aim = HELMSMAN_METERS_AIM;			// Aim HELMSMAN_METERS_AIM meters ahead
	double helmsmanFactor = HELMSMAN_FACTOR_AIM;	// The fraction of distance the Helmsman should aim for
	double distChangeaim = helmsmanFactor*m_aim; 	// When closer than 4 meters to waypoint, change to other Helmsman
	double kpLinVel = HELMSMAN_KP_LIN;				// gain for linear velocity P-controller
	double kpAngVel = HELMSMAN_KP_ANG;				// gain for angular velocity P-controller
	
	/* Direction vector of the line segment */
	double rx,ry;									// direction vector
	double urx,ury;									// Unit direction vector
	double rLen;									// Length of direction vector
	double s;										// Length from start point to projection point
	double xP,yP;									// Projection point xP and yP
	double xTarget, yTarget;						// The target point

	/* Angles */
	double thetaRef, thetaError;
	
	/* Do control */
	while ( ros::ok() ) {
				
		/* Run the controller if: correct control mode && more than a distance threshold from the end point && start and end point are not the same */
		if ( controlMode == CONTROL_MODE_HELMSMAN && ( sqrt(pow(lineSegment.end.x - states.x,2) + pow(lineSegment.end.y - states.y,2)) > HELMSMAN_ENDPOINT_THRESHOLD ) && (lineSegment.start.x != lineSegment.end.x || lineSegment.start.y != lineSegment.end.y ) ) {
	
			/*** Calculate the target point on the line ***/
			/* Defining trajectory vector(r = to - from) */
			rx = lineSegment.end.x - lineSegment.start.x;
			ry = lineSegment.end.y - lineSegment.start.y;
			
			/* Defining unit trajectory vector(ur = (r)/norm(r)) */
			rLen = sqrt(rx * rx + ry * ry);
			urx = rx / rLen;
			ury = ry / rLen;
			
			/* proj = from + ((pos - from)'*r/(r'*r))*r */
			s = ( (states.x-lineSegment.start.x)*rx+(states.y-lineSegment.start.y)*ry ) / ( rx * rx + ry * ry ); // s = ( (current.x-start.x)*r.x+(current.y-start.y)*r.y ) / ( r.x * r.x + r.y * r.y );
			xP = lineSegment.start.x + s*rx;
			yP = lineSegment.start.y + s*ry;
			
			/* If the distance from the projection to the endpoint is grater than the distChangeaim threshold, aim m_aim ahead of the projection point, else aim 1/helmsmanFactor ahead of the projection point */
			if ( sqrt(pow(xP-lineSegment.end.x,2) + pow(yP-lineSegment.end.y,2) ) > distChangeaim ) {
				xTarget = xP + urx*m_aim;
				yTarget = yP + ury*m_aim;
			} else {
				xTarget = xP + ((lineSegment.end.x-xP)/helmsmanFactor);
				yTarget = yP + ((lineSegment.end.y-yP)/helmsmanFactor);
			}
					
			/**** Calculate the reference angle ****/
			thetaRef = atan( (yTarget-states.y) / (xTarget-states.x) );		//maube use the atan2 function
	
			if ( (xTarget - states.x) < 0){
				thetaRef = thetaRef + M_PI ;	
			}
			else if( ((xTarget - states.x)> 0 ) && ( (yTarget - states.y) < 0) ){
				thetaRef = thetaRef + 2*M_PI;
			}
			
			/* Compute error angle ( difference between current angle and desired angle ) */
			thetaError = thetaRef - states.theta;
			
			/* Take care of 2pi jumps in error (forse error to be within +/- pi) */
			if( (fabs(thetaError) > M_PI) ){
				if (thetaError < 0){
					thetaError = thetaError + 2*M_PI;
				} else {
					thetaError = thetaError - 2*M_PI;
				}
			}

			/* Calculate the controller outputs */
			linear = sqrt( pow(lineSegment.end.x-states.x,2) + pow(lineSegment.end.y-states.y,2) ) * kpLinVel;
			angular = thetaError * kpAngVel;				
		
			/* Publish the calculated velocity */	
			publishVelocity(linear,angular);
			
		} else if ( controlMode == CONTROL_MODE_HELMSMAN && sqrt(pow(lineSegment.end.x - states.x,2) + pow(lineSegment.end.y - states.y,2)) < HELMSMAN_ENDPOINT_THRESHOLD ) {
		
			/* Set the start and endpoint of the linesegment to be (0,0) to disable without changing the control mode */
			lineSegment.start.x = 0;
			lineSegment.start.y = 0;
			lineSegment.end.x = 0;
			lineSegment.end.y = 0;
				
			/* Publish zero velocity to stop the vehicle */
			publishVelocity(0.0,0.0);
						
		}
		
		/* Spin and sleep */
		ros::spinOnce(); 
		r.sleep();
	}
	
}

/* The callback for setting the control mode */
void HelmsmanControllerNode::controlModeCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg) {
	
	std_msgs::String aString;
	std::stringstream temp;
	
	/* If the controller is started and was previously disabled make a countdown from 5 */
	if ( ctrl_mode_msg->data == CONTROL_MODE_HELMSMAN && controlMode == CONTROL_MODE_OFF ) {
		for ( unsigned int i = 5 ; i > 0 ; i-- ){
			temp.str("");
			temp << " Controller is starting in " << i << " seconds";
			ROS_INFO_STREAM("ROBUROC4_HELMSMAN_CONTROLLER:" << temp.str());
			
			aString.data = temp.str();
			stringPub_.publish(aString);
			
			sleep(1);		
		}
	}
	
	/* Set the new control mode */
	controlMode = ctrl_mode_msg->data ;

	/* If the new control mode is off, publish a zero velocity! */
	if ( controlMode == CONTROL_MODE_OFF ) {
		 publishVelocity(0.0,0.0);
	}

}

/* The callback for updating the line segment to follow*/
void HelmsmanControllerNode::lineSegmentCallback(const roburoc4::LineSegment::ConstPtr& lSeg){

	/* Update the line segment */
	lineSegment.start = lSeg->start;
	lineSegment.end = lSeg->end;

}

/* Function to saturate velocities to be within the limits of the robot */
void HelmsmanControllerNode::publishVelocity(double linear,double angular) {

	/* Velocity message */
	geometry_msgs::Twist velocity;
	
	/* Make limitations to the velocity */	
	velocity.linear.x = max(HELMSMAN_MIN_LINEAR_VEL,min(linear,HELMSMAN_MAX_LINEAR_VEL));
	velocity.angular.z = max(-HELMSMAN_MAX_ABS_ANGULAR_VEL,min(angular,HELMSMAN_MAX_ABS_ANGULAR_VEL));
	
	/* Publish the velocity */
	velPub_.publish(velocity);
		
}



/* The main function starts the controller */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "roburoc4_helmsmann_controller");
	HelmsmanControllerNode helmsmanControllerNode;

	helmsmanControllerNode.controlLoop();

	return 1;
}
