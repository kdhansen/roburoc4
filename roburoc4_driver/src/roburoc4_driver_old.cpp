#include <ros/ros.h>				// Ros declerations
#include <ros/console.h>			// DEBUG COMMANDS

#include <geometry_msgs/Twist.h>	// Message type to contain linear velocity
#include <boost/thread.hpp>			// Generate a tread to listen for incoming data
#include <roburoc4Comm.hpp>	

/* Define the Roburoc4 class */
class Roburoc4 {

	public:
		Roburoc4();
		~Roburoc4();
		
		void receiveLoop();

	private:
		/* Callback for request of velocities */
		void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVel);
		
		/* RobuROC4 communicator to be able to communicate with the robot */
		Roburoc4Comm* roburoc4Comm;
		
		/* Thread to handle the incoming messages from the socket */
		boost::thread* socketListener;

		/* ROS node handles, subscribers and publishers */ 
		ros::NodeHandle nh_;
		ros::Subscriber cmdVelSubscriber_;
		ros::Publisher curVelPublisher_;
		
};


Roburoc4::Roburoc4() {
	
	std::string serverAddress;
	int serverPort;
	
	/* Get the parameters from the parameter server */
	nh_.param<std::string>("/Roburoc4/server_address",serverAddress,"192.168.0.2");
	nh_.param("/Roburoc4/server_port",serverPort,60000);
	
	/* Setup the subscriber */
	cmdVelSubscriber_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1 , &Roburoc4::cmdVelCallback, this);
	
	/* Setup the publisher */
	curVelPublisher_ = nh_.advertise<geometry_msgs::Twist>("roc4/cur_vel", 1);

	/* Connect to the robot via a roburoc4Comm */
	roburoc4Comm = new Roburoc4Comm(serverAddress, serverPort);	
	
	/* Ask the robot to return velocity periodically */
	roburoc4Comm->subscribe(PURE_SERVICE_ID_DIFFERENTIAL,5);		// Differentiel and period of 5*10ms --> 20Hz
	roburoc4Comm->subscribe(PURE_SERVICE_ID_BATTERY,200);			// Differentiel and period of 200*10ms --> 1/2Hz
	
	/* Startup the thread for listening on the socket */
	socketListener = new boost::thread( boost::bind( &Roburoc4::receiveLoop, this ) );

}

Roburoc4::~Roburoc4() {

	/* Delete the thread listening to the socket */
	delete socketListener;

	/* Delete the connection to the robot */
	delete roburoc4Comm;
	
}


/* The loop used to get data from the communicator */
void Roburoc4::receiveLoop(){
	
	// TODO: Everything in here whould be moved somware else!
	
	
	
	ROS_DEBUG_STREAM("Roburoc4Node: Receiving thread is alive!");
	
	/* String stream for the input */
	std::stringstream ss_in;
	std::stringstream ss_tmp;
	
	/* The input archive for deserilization */	
	boost::archive::binary_iarchive ia(ss_in,1);
			
	/* While ros is okay continue to get messeages */
	while (ros::ok()){
		
		
		
		
		
		
		
		/* Get a string from the robot */
		std::string inputMessage = roburoc4Comm->getMessage();

		/* Deserialize to Telegram telegram first to get type */
		Telegram telegram;
		ss_in.str(""); ss_in << inputMessage.substr(0,1); // The first byte
		ia >> telegram;
		
		/* ERROR */
		if (telegram.getHeader() == 0x00) {
				ROS_DEBUG_STREAM("Roburoc4Node: Error message received");
		}
		
		/* Status update */
		else if (telegram.getHeader() == 0xFF) {
			
			StatusTelegram statusTelegram;
			ss_in.str(""); ss_in << inputMessage.substr(0,11);	// The first two bytes
			ia >> statusTelegram;
			
			ss_in.str(""); ss_in << inputMessage;				// The complete message			
			
			//~ /* Print the bytes if the command received is a status */
			//~ for ( unsigned int i = 0 ; i < inputMessage.size() ; i++ ){
				//~ printf("%02x ",(unsigned char)inputMessage.at(i));
			//~ } 
			//~ std::cout << std::endl;
			
			
			if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_BATTERY ) {
				
				BatteryStatusTelegram batteryStatusTelegram;
				ia >> batteryStatusTelegram;
				ROS_DEBUG_STREAM("Roburoc4Node: Battery status: state: " << (int)batteryStatusTelegram.getState() << " Remaining: " << (int)batteryStatusTelegram.getRemaining());
					
			}
			else if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_DIFFERENTIAL ) {
				
				DifferentialStatusTelegram differentialStatusTelegram;
				ia >> differentialStatusTelegram;
				//ROS_DEBUG_STREAM("Roburoc4Node: Differential TL: " << differentialStatusTelegram.getTargetLinearSpeed() << " TA: " << differentialStatusTelegram.getTargetAngularSpeed() << " CL: " << differentialStatusTelegram.getCurrentLinearSpeed() << " CA: " << differentialStatusTelegram.getCurrentAngularSpeed() );
								
			
				/* Publish the current velocity to the topic */
				geometry_msgs::Twist curVel;
				curVel.linear.x =  differentialStatusTelegram.getCurrentLinearSpeed() * 1.3; 			// Compensate for larger wheels
				curVel.angular.z = differentialStatusTelegram.getCurrentAngularSpeed() * 1.3 * 0.53;	// Compensate for larger wheels
				curVelPublisher_.publish(curVel);	
			
				
			}
			else if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_LOCALIZATION )  {
				
				LocalizationStatusTelegram localizationStatusTelegram;
				ia >> localizationStatusTelegram;
				ROS_DEBUG_STREAM("Roburoc4Node: Local status: ");
							
			}
			
					
		}
		else {
			//ROS_DEBUG_STREAM("Roburoc4Node: Response to command received");			
		}			

	}
	
}

/* The velocity callback to be able to set the velocity of the robot */
void Roburoc4::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVel){
		
	/* Send the velocity to the robot */
	roburoc4Comm->setVelocity((float)cmdVel->linear.x,(float)cmdVel->angular.z);	
	
}


/* The main function of the node */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "roburoc4_driver");
	
	/* Start the roburoc4 class */
	Roburoc4 *roburoc4 = new Roburoc4();
		
	/* Listen for callbacks */
	ros::spin();

	/* Delete the Roburoc4 */
	delete roburoc4;
	
	return 0;	
}
