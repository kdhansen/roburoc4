/*
 * By Rune Madsen and group 12gr832, Aalborg University, Nov. 2012
 * 
 * A joystick teleoperations node for differential drive vehicles. Made with inspiration from 
 * Melonee Wise's turtlebot_apps - joystick teleop node. And from teleop_joy by Karl Damkjær Hansen.
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopPS3Controller
{
  public:
    	TeleopPS3Controller();
    	void PublisherLoop();

  private:
	/* The ros node handle */
	ros::NodeHandle nh_;

	/* Callback for joystick input */	
 	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);	

	/* Variable for setting up the controller mode */
	int controllerMode_;

	/* Scaling factors ( maximum outputs ) */
	double angularScale_, linearScale_;

	/* Definition of buttons */
	int deadManButton_;
    	
	/* Make a message to contain the output velocity */
	geometry_msgs::Twist vel_;

	/* half of the width of the vehicle to control */
	double c_;

	/* The publisher and subscriper */
	ros::Subscriber joySub_;
	ros::Publisher velPub_;  
};


TeleopPS3Controller::TeleopPS3Controller():
  deadManButton_(13),
  angularScale_(5),
  linearScale_(2),
  c_(0.34),
  controllerMode_(1)
{
  nh_.param("/teleop_ps3_controller/angularScale", angularScale_, angularScale_);
  nh_.param("/teleop_ps3_controller/linearScale", linearScale_, linearScale_);
  nh_.param("/teleop_ps3_controller/deadManButton", deadManButton_, deadManButton_);
  nh_.param("/teleop_ps3_controller/controllerMode",controllerMode_, controllerMode_);

  joySub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopPS3Controller::joyCallback, this);

  velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  /* Set the initial velocity to zero */
	vel_.angular.z = 0;
	vel_.linear.x = 0;

}

void TeleopPS3Controller::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	/* Generate a zero velocity */
	static geometry_msgs::Twist zeroVel;
  	zeroVel.angular.z = 0;
  	zeroVel.linear.x = 0;
 
	/* If the controller mode is two stick mode */ 
	if (controllerMode_ == 2){
  		vel_.angular.z = ((joy->axes[3] - joy->axes[1])/(2)) * angularScale_; 
  		vel_.linear.x = ((joy->axes[3] + joy->axes[1])/2) * linearScale_;
  	} else if (controllerMode_ == 1 ) {
		vel_.angular.z = joy->axes[2] * angularScale_;
		vel_.linear.x = joy->axes[3] * linearScale_;
	}
	
	/* If the dead man botton is not pressed, set the velocity to be sent to zero */
	if ( joy->axes[deadManButton_] > - 0.5) {
		vel_ = zeroVel;
	}

}

void TeleopPS3Controller::PublisherLoop(){

	/* Define the loop rate */
	ros::Rate r(20); // 20 hz
	
	/* Run with the defined loop rate as long as ros is working */
	while (ros::ok())
	{
		ros::spinOnce();		// Spin to get new data
		velPub_.publish(vel_);	// Publish the data
		r.sleep();				// Sleep for a while
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ps3_controller_node");
	TeleopPS3Controller teleopPS3Controller;

	teleopPS3Controller.PublisherLoop();

	return 1;
}

