/***********************************************************************
 * RobuROC4 trajectory controller.
 * Based on work of projectgroup 12gr832. Main idea is to follow a trajectory defined by position (x,y,theta) 
 * and velocity (both angular and linear). For more information about the controller principle refer to 
 * roburoc4_documentation/project_reports/12gr832_robotic_road_striper.pdf
 * ********************************************************************/

/* ROS things */
#include <ros/ros.h>
#include <ros/console.h>

/* Messages */
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <roburoc4/Trajectory.h>
#include <roburoc4/TrajectoryPoints.h>
#include <roburoc4/States.h>

/* Common Controller Defines */
#include <roburoc4_controllers.hpp>

using namespace std;

class TrajectoryControllerNode {
	
	public:
		TrajectoryControllerNode();
		~TrajectoryControllerNode();
		
		void controlLoop();
		
	private:
		/* The ros node handle */
		ros::NodeHandle nh_;
		
		/* Publishers */
		ros::Publisher velPub_;
		ros::Publisher stringPub_;				// A string!
		
		/* Subscriber */
		ros::Subscriber trajectorySub_;
		ros::Subscriber stateSub_;
		ros::Subscriber controlModeSub_;
		
		/* Callback function for receiving the trajectory and current states */
		void trajectoryCallback(const roburoc4::Trajectory::ConstPtr& trajectory);
		void statesCallback(const roburoc4::States::ConstPtr& newStates);
		void controlModeCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg);
		
		/* Function to saturate the velocities such that they are within the limits of the RobuROC4 and publish it */
		void publishVelocity(double linear,double angular);
		
		/* The current trajectory */
		vector<roburoc4::TrajectoryPoints> points;
		vector<roburoc4::TrajectoryPoints>::iterator pointsIt;		
		
		/* Current states */
		roburoc4::States states;
		
		/* Control mode off / helmsmann / trajectory */
		unsigned char controlMode;	
};

TrajectoryControllerNode::TrajectoryControllerNode() {
	
		/* Velocity publisher */
		velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		/* Publish a string to the menu */
		stringPub_ =  nh_.advertise<std_msgs::String>("roburoc4_printString", 10);
		
		/* Subscriber for the trajectory */
		trajectorySub_ = nh_.subscribe<roburoc4::Trajectory>("roburoc4_controllers/trajectory", 1 , &TrajectoryControllerNode::trajectoryCallback, this);
		
		/* Subscriber for the states */
		stateSub_ = nh_.subscribe<roburoc4::States>("roburoc4_state_estimator/states", 1 , &TrajectoryControllerNode::statesCallback, this);
		
		/* Subscriber for the controlMode */
		controlModeSub_ = nh_.subscribe<std_msgs::UInt8>("roburoc4_controllers/controlMode", 1 , &TrajectoryControllerNode::controlModeCallback, this);
		
		/* Set the controller to be disabled by default */
		controlMode = CONTROL_MODE_OFF;
		
}

TrajectoryControllerNode::~TrajectoryControllerNode() {

}

/* The control loop used when the controller is running */
void TrajectoryControllerNode::controlLoop() {

	/* Define control rate */
	ros::Rate r(20);
	
	
	/* Controller Gains. */
	double Ka;
	double K[4];
	
	Ka = 2;
 	K[0] = 1;
	K[1] = 3.1;
	K[2] = 0.9;
	K[3] = 0;

	/* Velocity message to be published */
	geometry_msgs::Twist velocity;
	
	/* Trajectory "states" (Reference) */
	double xR, yR, thetaR;
	double vR, wR;
	
	/* Direction vectors */
	double qr_x, qr_y;		// Direction of reference coordinate system (trajectory)
	double q_x, q_y;		// Direction of the current states;
	
	/* Coordinates of the states in the reference coordinate system */
	double xa, xp;			// The position
	double qa, qp;			// The orientation
	
	/* Variables for the calculated velocities */
	double linear, angular;
	
	/* Do control */
	while ( ros::ok() ) {
		
		/* If the control mode allows this controller to publish velocities */
		if ( controlMode == CONTROL_MODE_TRAJECTORY ) {
			
			/* If there exists a trajectory, and we havent reached the last point on the trajectory */
			if ( !points.empty() && pointsIt != points.end() ) {
				
				/* Extract needed variables from the trajectory */
				xR = (*pointsIt).pose.x;
				yR = (*pointsIt).pose.y;
				thetaR = (*pointsIt).pose.theta;
				
				vR = (*pointsIt).velocity.linear.x;
				wR = (*pointsIt).velocity.angular.z;			
				
				/* Calculate the heading vector for the current STATE from the heading */
				q_x = cos(states.theta);								//~ q = [cos(xi(3,k)) sin(xi(3,k))]';
				q_y = sin(states.theta);
				
				/* Calculate the heading vector for the current REFERENCE */
				qr_x = cos(thetaR);										//~ qr = [cos(xr(3,k)) sin(xr(3,k))]';
				qr_y = sin(thetaR);
					
				/* Calculate the errors in the reference coordinate system */
				xa = (states.x - xR) * qr_x + (states.y - yR) * qr_y;	//~ xa(k) = (xi(1:2,k) - xr(1:2,k))'*qr;
				xp =-(states.x - xR) * qr_y + (states.y - yR) * qr_x;	//~ xp(k) = (xi(1:2,k) - xr(1:2,k))'*R*qr;
				qa =    (q_x - qr_x) * qr_x +    (q_y - qr_y) * qr_y; 	//~ qa(k) = (q - qr)'*qr;
				qp =   -(q_x - qr_x) * qr_y +    (q_y - qr_y) * qr_x;	//~ qp(k) = (q - qr)'*R*qr;
					
				/* Calculate the inputs to the system */
				linear = - Ka*xa - qa*vR - xp*wR + vR;					//~ ui(1,k) = -Ka*xa(k) - qa(k)*xr(6,k) - xp(k)*xr(4,k) + xr(6,k);
				angular = - K[0]*xp - K[1]*qp - K[2]*states.theta - K[3]*states.q1 + wR;
				
				/* Go to the next trajectory point */
				pointsIt++;
						
				/* Filter and publish the velocity */
				publishVelocity(linear,angular);
									
			}
			
			/* If the endpoint is reached */
			else if ( pointsIt == points.end() ) {
				
					/* clear the list of points */
					points.clear();
					
					/* Publish a zero velocity */
					publishVelocity(0,0);
					
				
			}			
			
		} 
		
		/* If the controller is completely disabled and there exists points in the list, print the Distance to the next point */
		else if ( controlMode == CONTROL_MODE_OFF && !points.empty() ) {
			double dist =sqrt(pow( ( states.x - (*pointsIt).pose.x ) ,2) + pow( ( states.y - (*pointsIt).pose.y ) ,2));
			ROS_DEBUG_STREAM( "ROBUROC4_TRAJECTORY_CONTROLLER: Distance to the next point: " << dist << " distX = " <<  states.x - (*pointsIt).pose.x  << " distY = " << states.y - (*pointsIt).pose.y);
		}
		
		/* Sleep and spin to see for any new data on the topics */
		r.sleep();
		ros::spinOnce(); 
	}
	
}


/* Callback for updating the trajectory */ 
void TrajectoryControllerNode::trajectoryCallback(const roburoc4::Trajectory::ConstPtr& trajectory){
		
	/* Save the new trajectory (points) */
	points = trajectory->points;
	
	/* Find the first point on the trajectory that we want to track:
	 * The first point on the trajectory where the distance from the current state to the point on the trajectory decreases */
	
	ROS_DEBUG_STREAM("ROBUROC4_TRAJECTORY_CONTROLLER: Searching for start point on trajectory");
	
	double dist, distPrev = 0;
	unsigned char first = 1;
	
	for ( pointsIt = points.begin() ; pointsIt < points.end() ; pointsIt++ ){
		
		/* Calculate the current distance from the state of the vehicle to the first trajectory point */
		dist = pow( ( states.x - (*pointsIt).pose.x ) ,2) + pow( ( states.y - (*pointsIt).pose.y ) ,2);
			
		/* If it is not the first point see if the distance has increased */
		if ( first != 1 ) {
			/* If the distance has increased break out of the for loop */
			if ( dist > distPrev ) {
				break;
			}
		}
		
		/* Indicate that the next point is not the first */
		first = 0;
		
		/* Save the new calculated distance as the previous distance */
		distPrev = dist;		
	}
	
	ROS_DEBUG_STREAM("ROBUROC4_TRAJECTORY_CONTROLLER: Point found!");
	
	if (pointsIt == points.end())
		ROS_DEBUG_STREAM("ROBUROC4_TRAJECTORY_CONTROLLER:  -> As the last point");
}


/* Callback to update the local copy of the states */
void TrajectoryControllerNode::statesCallback(const roburoc4::States::ConstPtr& newStates){
	
	/* Update the local copy of the states */
	states.x = newStates->x;
	states.y = newStates->y;
	states.theta = newStates->theta;
	states.omega = newStates->omega;
	states.q1 = newStates->q1;

}


/* The callback for setting the control mode */
void TrajectoryControllerNode::controlModeCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg) {

	std_msgs::String aString;
	std::stringstream temp;

	/* If the controller is started and was previously disabled make a countdown from 5 */
	if ( ctrl_mode_msg->data == CONTROL_MODE_TRAJECTORY && controlMode == CONTROL_MODE_OFF ) {
		for ( unsigned int i = 5 ; i > 0 ; i-- ){
			temp.str("");
			temp << "Controller is starting in " << i << " seconds";
			
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_CONTROLLER: " << temp.str());
			
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

/* Function to saturate velocities to be within the limits of the robot */
void TrajectoryControllerNode::publishVelocity(double linear, double angular){

	/* Velocity message */
	geometry_msgs::Twist velocity;
	
	linear = max(-2.6*1.3,min(linear,2.6*1.3));
	angular = max(-6.5*1.3,min(angular,6.5*1.3));
	
	velocity.linear.x = linear;
	velocity.angular.z = angular;
	
	/* Publish the velocity */
	velPub_.publish(velocity);
}


/* The main function starts the controller */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "roburoc4_trajectory_controller");
	TrajectoryControllerNode controllerNode;

	controllerNode.controlLoop();

	return 1;
}
