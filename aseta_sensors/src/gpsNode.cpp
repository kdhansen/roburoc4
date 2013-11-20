#include <ros/ros.h>  				// Ros declerations
#include <ros/console.h>			// DEBUG COMMANDS

#include <sensor_msgs/NavSatFix.h>	// Message types for the GPS position output
#include <sensor_msgs/NavSatStatus.h>

#include <geometry_msgs/Twist.h>	// Message type to contain linear velocity
#include <geometry_msgs/Pose2D.h>	// Messate type to contain heading

#include <serialCommunicator.hpp> 	// Serial communication
#include <helper_functions.hpp>		// Helper functions for splitting a string etc.

#include <string>					// std::string


using namespace std;

class GPSNode {
	
	private:
	/* ROS fields */
	ros::NodeHandle nh_; 				// Node handle
	
	ros::Publisher positionPub_;		// Publisher for the position
	ros::Publisher orientationPub_;		// Publisher for the orientation
	ros::Publisher velPub_;				// Publisher for the velocity
	
	/* Serial Communication fields */
	serialCommunicator* serialCom_;		// Serial communicator
	string serialPort_;					// The serial port name where the GPS is connected
	int serialBaud_;					// The serial port baud rate
	
	public:
	/* Constructor */
	GPSNode();
	~GPSNode();
		
	/* Main loop function */
	void loop();
};

GPSNode::GPSNode():
	serialPort_("/dev/ttyUSB1"),
	serialBaud_(115200)
 {
	
	/* Get the parameters from the parameter server */
	nh_.param<std::string>("/GPSNode/serial_port_name",serialPort_,serialPort_);
	nh_.param("/GPSNode/serial_port_baud",serialBaud_,serialBaud_);

	/* Disp debug information About the serial port*/
	ROS_INFO_STREAM("GPS NODE: Serial port: " << serialPort_);
	ROS_INFO_STREAM("GPS NODE: Baud rate: " << serialBaud_);
	
	/* Connect to the serial communicator */
	try{
		serialCom_ = new serialCommunicator(serialPort_,serialBaud_);
	} catch (std::string message) {
		ROS_FATAL_STREAM( "GPS NODE: Error creating the serialCommunicator: " << message );
		exit(0);	// Close the node!
	}
	
	/* Setup the GPS to print out the correct format*/
	// TODO: Make this setup
		
	/* Publisher: */
	positionPub_ = nh_.advertise<sensor_msgs::NavSatFix>("GPS/position", 10);
	orientationPub_ = nh_.advertise<geometry_msgs::Pose2D>("GPS/orientation", 10);
	velPub_ = nh_.advertise<geometry_msgs::Twist>("GPS/velocity", 10);
	
	/* Disp debug information that the node has started*/
	ROS_INFO("GPS NODE: GPSNode started");
	
}

GPSNode::~GPSNode() {
	
	/* Delete the serial communicator */
	delete serialCom_;
		
}

/* The loop for receiving data from the GPS */
void GPSNode::loop(){
	
	/* Get a single line from the serial port */
	string serialMessage = serialCom_->getLine();
	
	/* If the line does not contain any data, skip the rest and read for new data
	 *  - length must be above 30 chars shortest message is about 33 chars
	 *  - Message must contain '$PASHR,POS'
	 *  - Message must contain '*' 
	 * */
	if ( serialMessage.length() > 30 && serialMessage.find("$PASHR,POS") < serialMessage.length() && serialMessage.find("*") < serialMessage.length() ) {
		
		/* Remove everything before '$' */
		serialMessage = serialMessage.substr(serialMessage.find("$"));
								 
		/* Check if the received string has the correct format (checksum check) */
		if ( calculateNMEAChecksum(serialMessage) !=  serialMessage.substr(serialMessage.find("*")+1,2) ) {	// If not, write an error message
			ROS_INFO_STREAM("GPS NODE: Checksum: (" << calculateNMEAChecksum(serialMessage) << ") EXPECTED: (" << serialMessage.substr(serialMessage.find("*")+1,2) << ")");
			
		} else {	// Else: use the received data 
			
			/***
			* If the messages enters here it has the following format
			* $PASHR,POS,2,10,100835.35,5700.9160300,N,00959.1047863,E,045.936,0.4,000.0,000.009,-000.003,1.7,0.9,1.4,0.9,Hp23*04
			* 
			* $PASHR,POS,d1,d2,m3,m4,c5,m6,c7,f8,f9,f10,f11,f12,f13,f14,f15,f16,s17*cc
			* Parameter
			* d1 	Position mode:
			* 		0: Autonomous 
			* 		1: RTCM code differential or SBAS differential
			* 		2: RTK float
			* 		3: RTK fixed
			* 		5: Extrapolated
			* d2	Count of satellites used in position computation: 	0-26
			* m3	Current UTC time of position (hhmmss.ss)			000000.00-235959.99
			* m4	Latitude of position (ddmm.mmmmmm)			 		dd: 0-90° , mm.mmmmmm: 00-59.999999 minutes
			* c5	North (N) or South (S)								N,S
			* m6	Longitude of position (dddmm.mmmmmm)			 		dd: 0-180° , mm.mmmmmm: 00-59.999999 minutes
			* c7	East (E) or West (W)								E,W
			* f8	Altitude above the WGS84 ellipsoid					±9999.000
			* f9	Age of Differential data, in seconds				0.0-600.0
			* f10	True Track/Course Over Ground, in degrees			0.0-359.9
			* f11	Speed Over Ground, in knots							0.0-999.9
			* f12	Vertical velocity in dm/s							±999.9
			* f13	PDOP												0-99.9
			* f14	HDOP												0-99.9
			* f15	VDOP												0-99.9
			* f16	TDOP												0-99.9
			* s17	Firmware version									4-char string
			* *cc	Checksum											*00-*FF
			***/
			
			/* Remove the $PASHR,POS heading and the ,*<checksum> tail */
			serialMessage = serialMessage.substr(serialMessage.find("$PASHR,POS")+11);
			serialMessage = serialMessage.substr(0,serialMessage.rfind('*'));
			
			/* Split the string by the commas using the split function from the helper functions */
			vector<string> fields;
			split(serialMessage,',',fields);				

			/* Test if the correct number of fields in the string */
			if ( fields.size() != 17 ) {
				ROS_WARN_STREAM("GPS NODE: Wrong number of fields in string: " << fields.size() << " (17)");
			} else {

				/** Extract relevant data from the datastring **/
				
				/* The position */
				sensor_msgs::NavSatFix position;
				sensor_msgs::NavSatStatus positionStatus;
				
				/* Extract position mode */		
				positionStatus.status = (unsigned char) atoi(fields[0].c_str());
				
				/* Setup the service to på GPS  (Use to indicate number of satelites. This is not the correct way to do it but is left as this to help the developer see the number of sattelites by watching the GPS topic) */
				positionStatus.service = (unsigned char) atoi(fields[1].c_str());

				/* Insert the position status into the position message */
				position.status = positionStatus;
				
				/* Test if the field contaning the latitude is longer than 3 chars, indicating that a position is found */
				if ( fields[3].length() > 3 ) {

					/* Extract latitude ( and convert into decimal degrees + use sign to indicate N/S) */
					position.latitude = (atof((fields[3].substr(0,2)).c_str()) + atof((fields[3].substr(2)).c_str()) / 60) * (1 - 2 * (int)(fields[4]=="S"));
				
					
					/* Extract longitude ( and convert into decimal degrees + use sign to indicate E/W) */
					position.longitude =  (atof((fields[5].substr(0,3)).c_str()) + atof((fields[5].substr(3)).c_str()) / 60) * (1 - 2 * (int)(fields[6]=="W"));
				
				} else {
					position.latitude = 0;
					position.longitude = 0;
				}
				
				/* Extract altitude */
				position.altitude = atof(fields[7].c_str());

				/* Publish the position */
				positionPub_.publish(position);
				

				/* If the latitude is not zero (the GPS has satelites), publish heading and velocity */
				if ( position.latitude != 0 ) {
					
					/* The orientation */
					geometry_msgs::Pose2D orientation;
					
					/* Extract the orientation */
					// orientation.theta = atof(fields[9].c_str());
					orientation.theta = fmod((90.0 - atof(fields[9].c_str())) * M_PI / 180.0, 2 * M_PI );	// Turn into desired coordinate system
					
					/* Publish the orientation */
					orientationPub_.publish(orientation);
					
					/* The velocity */
					geometry_msgs::Twist velocity;
					
					/* Extract the orientation in knot and convert to m/s */
					velocity.linear.x = atof(fields[10].c_str()) * 0.514444444;
					
					/* Publish the orientation */
					velPub_.publish(velocity);
				}				
				
			}					
		}
	}	
}


/* Main function */
int main(int argc, char *argv[]){
	
	/* Initialise node */
	ros::init(argc, argv, "GPSNode");

	/* Generate the node object */
	GPSNode GPSNode;	

	/* While ros is okay */
	while (ros::ok()) {
		GPSNode.loop();	// Take a loop
	}

}



