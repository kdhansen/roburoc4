#include <ros/ros.h>  				// Ros declerations
#include <ros/console.h>			// DEBUG COMMANDS

#include <geometry_msgs/Twist.h>	// Message type to contain angular velocity

#include <serialCommunicator.hpp> 	// Serial communication
#include <helper_functions.hpp>		// Helper functions for splitting a string etc.

#include <string>					// std::string


using namespace std;

class GyroNode {
	
	private:
	/* ROS fields */
	ros::NodeHandle nh_; 				// Node handle
	
	ros::Publisher velPub_;				// Publisher for the rotational velocity
	
	/* Serial Communication fields */
	serialCommunicator* serialCom_;		// Serial communicator
	string serialPort_;					// The serial port name where the Gyro is connected
	int serialBaud_;					// The serial port baud rate			

	public:
	/* Constructor */
	GyroNode();
	~GyroNode();
		
	/* Main loop function */
	void loop();
};

GyroNode::GyroNode():
	serialPort_("/dev/ttyUSB0"),
	serialBaud_(9600)
 {
	
	/* Get the parameters from the parameter server */
	nh_.param<std::string>("/GyroNode/serial_port_name",serialPort_,serialPort_);
	nh_.param("/GyroNode/serial_port_baud",serialBaud_,serialBaud_);

	/* Disp debug information About the serial port*/
	ROS_INFO_STREAM("GYRO NODE: Serial port: " << serialPort_);
	ROS_INFO_STREAM("GYRO NODE: Baud rate: " << serialBaud_);
	
	/* Connect to the serial communicator */
	try{
		serialCom_ = new serialCommunicator(serialPort_,serialBaud_);
	} catch (string message) {
		ROS_FATAL_STREAM( "GYRO NODE: Error creating the serialCommunicator: " << message );
		exit(0);	// Close the node!
	}
		
	/* Publisher: */
	velPub_ = nh_.advertise<geometry_msgs::Twist>("Gyro/velocity", 10);

	/* Disp debug information that the node has started*/
	ROS_INFO("GYRO NODE: GyroNode started");
	
}

GyroNode::~GyroNode() {
	
	/* Delete the serial communicator */
	delete serialCom_;
	
}

/* The loop for receiving data */
void GyroNode::loop(){
	
	// Incoming data vars;
	int nChars;
	unsigned char c;

	// Decoded data
	unsigned short R = 0;
	unsigned char T = 0;
	unsigned char C = 0;
	unsigned short B = 0;
	unsigned char X = 0;
	
	// Temp variables and the counter for char number in the string
	unsigned short charCounter = 0;
	unsigned short tmpShort;
	unsigned char tmpChar;
	
	// Checksum
	unsigned int checksum = 0;
	
	// Temperature ( incl. flag );
	char tempFlag = 0;
	double temp = 0;

	while (ros::ok()) {
		
		/* Get a new char */
		nChars = serialCom_->getChar(&c);
		
		/* If a char is received, process the data */
		if (nChars == 1) { 
		
			/* Look for the HI syncbit. When it is recieved  reset the char counter*/
			tmpChar = c & 0x80;	
			if ( tmpChar ) {
				charCounter = 1;
			} else {
				charCounter++ ;
			}
			
			/* Do some action dependent on the character number (charCounter) */
			switch (charCounter) {
				
					case 1:
						X = c << 1;							// [~ X7 X6 X5 X4 X3 X2 X1] --> [ X7 X6 X5 X4 X3 X2 X1 0]
						break;
						
					case 2:			
						// extract the LSB of X from c
						tmpChar =((c & 0x40) >> 6);			// [ ~ X0 B15 B14 B13 B12 B11 B10 ] --> [ 0 X0 0 0 0 0 0 0 ]--> [ 0 0 0 0 0 0 0 X0]
						X = X + tmpChar;					// [ X7 X6 X5 X4 X3 X2 X1 0] + [ 0 0 0 0 0 0 0 X0] = [ X7 X6 X5 X4 X3 X2 X1 X0]
										
						// extract the MSB's B from c					
						B = (c << (2+8));					// [ ~ X0 B15 B14 B13 B12 B11 B10 ] --> [ B15 B14 B13 B12 B11 B10 0 0 0 0 0 0 0 0 0 0 ];
						break;
						
					case 3:
						// extract more bits of B from c
						tmpShort =((c & 0x7F) << 3);		// [ ~ B9 B8 B7 B6 B5 B4 B3 ] --> [ 0 0 0 0 0 0 B9 B8 B7 B6 B5 B4 B3 0 0 0 ];
						B = B + tmpShort;					// [ B15 B14 B13 B12 B11 B10 0 0 0 0 0 0 0 0 0 0 ] + [ 0 0 0 0 0 0 B9 B8 B7 B6 B5 B4 B3 0 0 0 ] = [ B15 B14 B13 B12 B11 B10 B9 B8 B7 B6 B5 B4 B3 0 0 0 ];
						break;
				
					case 4:
						// extract LSB's of B from c
						tmpShort = ((c & 0x70) >> 4);		// [ ~ B2 B1 B0 C7 C6 C5 C4 ] --> [ 0 0 0 0 0 0 0 0 0 0 0 0 0 B2 B1 B0 ];
						B = B + tmpShort;					// [ B15 B14 B13 B12 B11 B10 B9 B8 B7 B6 B5 B4 B3 0 0 0 ] + [ 0 0 0 0 0 0 0 0 0 0 0 0 0 B2 B1 B0 ] = [ B15 B14 B13 B12 B11 B10 B9 B8 B7 B6 B5 B4 B3 B2 B1 B0 ]
						
						// extract MSB's of C from c
						C = c << 4;							// [ ~ B2 B1 B0 C7 C6 C5 C4 ] --> [ C7 C6 C5 C4 0 0 0 0]
						break;
						
					case 5:
						// extract the LSB's of C from c
						tmpChar = ((c & 0x78) >> 3);		// [ ~ C3 C2 C1 C0 T7 T6 T5 ] --> [ 0 0 0 0 C3 C2 C1 C0 ]
						C = C + tmpChar;					// [ C7 C6 C5 C4 0 0 0 0] + [ 0 0 0 0 C3 C2 C1 C0 ] = [ C7 C6 C5 C4 C3 C2 C1 C0 ]
						
						// extract the MSB's of T from c
						T = c << 5;							// [ ~ C3 C2 C1 C0 T7 T6 T5 ] --> [ T7 T6 T5 0 0 0 0 0 ]
						break;
						
					case 6:
						// extract the LSB's of T from c	
						tmpChar = 	((c & 0x7C) >> 2);		// [ ~ T4 T3 T2 T1 T0 R15 R14] --> [ 0 0 0 T4 T3 T2 T1 T0]
						T = T + tmpChar;					// [ T7 T6 T5 0 0 0 0 0 ] + [ 0 0 0 T4 T3 T2 T1 T0] = [ T7 T6 T5 T4 T3 T2 T1 T0]
						
						// extract MSB's of R from C
						R = c << (6 + 8);					// [ ~ T4 T3 T2 T1 T0 R15 R14] --> [ R15 R14 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
						break;
						
					case 7:
						// extract more bits of R from c
						tmpShort = ((c & 0x7F) << 7);		// [ ~ R13 R12 R11 R10 R9 R8 R7 ] --> [ 0 0 R13 R12 R11 R10 R9 R8 R7 0 0 0 0 0 0 0]
						R = R + tmpShort;					// [ R15 R14 0 0 0 0 0 0 0 0 0 0 0 0 0 0] + [ 0 0 R13 R12 R11 R10 R9 R8 R7 0 0 0 0 0 0 0] = [ R15 R14 R13 R12 R11 R10 R9 R8 R7 0 0 0 0 0 0 0]
						break;
						
					case 8:
						//Extract the LSB's of R from c
						tmpShort = (c & 0x7F);				// [ ~ R6 R5 R4 R3 R2 R1 R0 ] --> [ 0 0 0 0 0 0 0 0 0 R6 R5 R4 R3 R2 R1 R0 ]
						R = R + tmpShort;					// [ R15 R14 R13 R12 R11 R10 R9 R8 R7 0 0 0 0 0 0 0] + [ 0 0 0 0 0 0 0 0 0 R6 R5 R4 R3 R2 R1 R0 ] = 0;				
						break;
			
			}
			
			
			/* If char number is 8, do checksum and ... */
			if (charCounter == 8 ) {
									
				// Calculate the checksum and check for validity of message;
				checksum = (R & 0x00FF) + (R >> 8) + T + C + (B & 0x00FF) + (B >> 8);
				checksum = (checksum % 256) - 1;	//TODO: Find out why this -1 is nessecary
				checksum = (unsigned char)(~checksum);
				
				/* If checksum is okay */
				if (checksum == X ){		
				
					/* Calculate Rotation */
					double rot = (short)R;
					rot = rot * 0.00305;			// 100 deg/s units
				
					/* Calculate Temperature */
					tmpChar = T & 0x80;				// See if this is temperature message 1 or 2;
					if (tmpChar){
						temp = T & 0x7F;
						tempFlag = 0;
					} else {
						temp = temp + (T << 7);
						tempFlag = 1;
					}			
				
					if (tempFlag == 1) {
						temp = temp * 0.05;			// Convert to degrees celcius
					}
									
					/* Publish the rotation */
					geometry_msgs::Twist velocity;
					velocity.angular.z = rot * M_PI / 180.0;	//Convert into radians/s before publishing
					velPub_.publish(velocity);									
					
				} else {
					ROS_DEBUG_STREAM("GYRO NODE Checksum error");
				}
			}
		}
	}
}


/* Main function */
int main(int argc, char *argv[]){
	
	/* Initialise node */
	ros::init(argc, argv, "GyroNode");

	/* Generate the node object */
	GyroNode GyroNode;	

	/* Loop */
	GyroNode.loop();

}



