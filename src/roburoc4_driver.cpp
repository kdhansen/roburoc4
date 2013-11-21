// Driver for the RobuROC4.
//
// Interfaces to the low-level controller on the RobuROC4.
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

#include <ros/ros.h>				// Ros declerations
#include <ros/console.h>			// DEBUG COMMANDS

#include <geometry_msgs/Twist.h>	// Message type to contain linear velocity
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Int32.h>	
#include <std_msgs/UInt8.h>	
#include <roburoc4/battery.h>
#include <roburoc4/io.h>
#include <roburoc4/telemeter.h>

#include <boost/thread.hpp>			// Generate a tread to listen for incoming data
#include <roburoc4Comm.hpp>	

/* Define the RobuROC4Driver class */
class RobuROC4Driver {

	public:
		RobuROC4Driver();
		~RobuROC4Driver();
		
		void receiveLoop();

	private:
		/* Callbacks */
		void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVel);
		void requestPropertiesCallback(const std_msgs::Int32::ConstPtr& serviceId); 
		void setDigitalOutputCallback(const std_msgs::UInt8::ConstPtr& out); 
		void setCurrentPositionCallback(const geometry_msgs::Pose2D::ConstPtr& pos); 
		
		/* RobuROC4 communicator to be able to communicate with the robot */
		Roburoc4Comm* roburoc4Comm;
		
		/* Thread to handle the incoming messages from the socket */
		boost::thread* socketListener;

		/* ROS node handles, subscribers and publishers */ 
		ros::NodeHandle nh_;
		
		ros::Subscriber cmdVelSubscriber_;
		ros::Subscriber requestPropertiesSubscriber_;
		ros::Subscriber requestDigitalOutputSubscriber_;
		ros::Subscriber setCurPosSubscriber_;
		
		ros::Publisher curVelPublisher_;
		ros::Publisher curPosPublisher_;
		ros::Publisher batInfoPublisher_;
		ros::Publisher ioPublisher_;
		ros::Publisher telemeterPublisher_;
		
		
		
};


RobuROC4Driver::RobuROC4Driver() {
	
	std::string serverAddress;
	int serverPort;
	
	/* Get the parameters from the parameter server */
	nh_.param<std::string>("/Roburoc4/server_address",serverAddress,"192.168.0.2");
	nh_.param("/Roburoc4/server_port",serverPort,60000);
	
	/* Setup the subscribers */
	cmdVelSubscriber_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1 , &RobuROC4Driver::cmdVelCallback, this);
	requestPropertiesSubscriber_ = nh_.subscribe<std_msgs::Int32>("roburoc4_driver/req_properties", 1, &RobuROC4Driver::requestPropertiesCallback, this);
	requestDigitalOutputSubscriber_ = nh_.subscribe<std_msgs::UInt8>("roburoc4_driver/set_digital_output",1,&RobuROC4Driver::setDigitalOutputCallback, this);
	setCurPosSubscriber_ = nh_.subscribe<geometry_msgs::Pose2D>("roburoc4_driver/set_cur_pos",1,&RobuROC4Driver::setCurrentPositionCallback, this);
	
	/* Setup the publisher */
	curVelPublisher_ = nh_.advertise<geometry_msgs::Twist>("roburoc4_driver/cur_vel", 1);
	curPosPublisher_ = nh_.advertise<geometry_msgs::Pose2D>("roburoc4_driver/cur_pos", 1);
	batInfoPublisher_ = nh_.advertise<roburoc4::battery>("roburoc4_driver/battery_info", 1);
	ioPublisher_ = nh_.advertise<roburoc4::io>("roburoc4_driver/io", 1);
	telemeterPublisher_ = nh_.advertise<roburoc4::telemeter>("roburoc4_driver/telemeter", 1);

	/* Connect to the robot via a roburoc4Comm */
	roburoc4Comm = new Roburoc4Comm(serverAddress, serverPort);	
	
	/* Ask the robot to return service messages periodically */
	ros::Duration(0.1).sleep();
	roburoc4Comm->subscribe(PURE_SERVICE_ID_DIFFERENTIAL,5);			// Differential info:	period of 5*10ms --> 20Hz
	
	ros::Duration(0.1).sleep();
	roburoc4Comm->subscribe(PURE_SERVICE_ID_TELEMETER,5);				// Telemeter info:		period of 5*10ms --> 20Hz
	
	ros::Duration(0.1).sleep();
	roburoc4Comm->subscribe(PURE_SERVICE_ID_IOCARD,5);					// IOCard inputs:		period of 5*10ms --> 20Hz
	
	ros::Duration(0.1).sleep();
	roburoc4Comm->subscribe(PURE_SERVICE_ID_LOCALIZATION,5);			// Localization:		period of 5*10ms --> 20Hz
	
	ros::Duration(0.1).sleep();
	roburoc4Comm->subscribe(PURE_SERVICE_ID_BATTERY,200);				// Battery Information:	period of 200*10ms --> 1/2Hz
	
	/* Startup the thread for listening on the socket */
	socketListener = new boost::thread( boost::bind( &RobuROC4Driver::receiveLoop, this ) );

}

RobuROC4Driver::~RobuROC4Driver() {

	/* Delete the thread listening to the socket */
	delete socketListener;

	/* Delete the connection to the robot */
	delete roburoc4Comm;
	
}


/* The loop used to get data from the communicator */
void RobuROC4Driver::receiveLoop(){

	ROS_DEBUG_STREAM("Roburoc4 Driver: Receiving thread is alive!");
	
	unsigned int msg_type;
	
	while (ros::ok()) {
		msg_type = roburoc4Comm->getNewTelegramType();	// Listen for new telegrams from the robot
		
		/* Switch on the telegram type */
		
		/* Status telegrams */
		if( msg_type == RR4COM_STATUS_IOCARD ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: STATUS IOCard");
					
					IOCardStatusTelegram ioStatus = roburoc4Comm->getIOCardStatusTelegram();
					
					/* Publish the IO card data */
					roburoc4::io ioData;
					ioData.AI0 = ioStatus.getAnalogInput(0);
					ioData.AI1 = ioStatus.getAnalogInput(1);
					ioData.AI2 = ioStatus.getAnalogInput(2);
					ioData.AI3 = ioStatus.getAnalogInput(3);
					ioData.AI4 = ioStatus.getAnalogInput(4);
					ioData.AI5 = ioStatus.getAnalogInput(5);
					ioData.AI6 = ioStatus.getAnalogInput(6);
					ioData.AI7 = ioStatus.getAnalogInput(7);
					ioData.AI8 = ioStatus.getAnalogInput(8);
					ioData.DI0 = ioStatus.getDigitalInput(0);
					ioData.DI1 = ioStatus.getDigitalInput(1);
					ioData.DI2 = ioStatus.getDigitalInput(2);
					ioPublisher_.publish(ioData);					
					
		}
		
		else if ( msg_type == RR4COM_STATUS_TELEMETER ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: STATUS Telemeter");
					
					TelemeterStatusTelegram teleStatus = roburoc4Comm->getTelemeterStatusTelegram();
					
					/* Publish telemeter status */
					roburoc4::telemeter telemeterData;
					telemeterData.range1 = teleStatus.getDist(0);
					telemeterData.range2 = teleStatus.getDist(1);
					telemeterPublisher_.publish(telemeterData);
		}
					
		else if ( msg_type == RR4COM_STATUS_BATTERY ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: STATUS Battery");
					
					BatteryStatusTelegram batStatus = roburoc4Comm->getBatteryStatusTelegram();
					
					/* Publish the information about the battery */
					roburoc4::battery batInfo;
					batInfo.state = batStatus.getState();
					batInfo.remaining = batStatus.getRemaining();
					batInfoPublisher_.publish(batInfo);
					
		}
					
		else if ( msg_type == RR4COM_STATUS_LOCALIZATION ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: STATUS Localization");
					
					LocalizationStatusTelegram localStatusTelegram = roburoc4Comm->getLocalizationStatusTelegram();
					
					/* Publish current Position */
					geometry_msgs::Pose2D curPos;
					curPos.x = localStatusTelegram.getX();
					curPos.y = localStatusTelegram.getY();
					curPos.theta = localStatusTelegram.getTheta();
					curPosPublisher_.publish(curPos);
								
		}
			
		else if ( msg_type == RR4COM_STATUS_DIFFERENTIAL ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: STATUS Differential");
					
					DifferentialStatusTelegram differentialStatusTelegram = roburoc4Comm->getDifferentialStatusTelegram();

					/* Publish the current velocity to the topic */
					geometry_msgs::Twist curVel;
					curVel.linear.x =  differentialStatusTelegram.getCurrentLinearSpeed() * 1.3; 			// Compensate for larger wheels by multiplying by 1.3
					curVel.angular.z = differentialStatusTelegram.getCurrentAngularSpeed() * 1.3;			//
					curVelPublisher_.publish(curVel);	
		}
		
		
		/* Property telegrams */
		else if ( msg_type == RR4COM_PROPERTY_IOCARD ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: PROPERTY IOCard");
					IOCardPropertiesTelegram ioProps = roburoc4Comm->getIOCardPropertiesTelegram();
					printf("\nIOCard properties:\n");
					printf("No Analog inputs: %d\n", ioProps.getNumberAnalogInputs());
					printf("No Analog outputs: %d\n", ioProps.getNumberAnalogOutpts());
					printf("No Digital inputs: %d\n", ioProps.getNumberDigitalInputs());
					printf("No Digital outputs: %d\n", ioProps.getNumberDigitalOutputs());
		}
					
		else if ( msg_type == RR4COM_PROPERTY_TELEMETER ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: PROPERTY Telemeter");
					TelemeterPropertiesTelegram teleProps = roburoc4Comm->getTelemeterPropertiesTelegram();
					printf("\nTelemeter properties:\n");
					printf("No telemeter devices: %d\n", teleProps.getNumberTelemeterServices());
					printf("1. X: %f\n",teleProps.getX(0));
					printf("1. Y: %f\n",teleProps.getY(0));
					printf("1. T: %f\n",teleProps.getT(0));
					printf("1. FoV: %f\n",teleProps.getFoV(0));
					printf("1. minDist: %f\n",teleProps.getminD(0));
					printf("1. maxDist: %f\n",teleProps.getmaxD(0));
					printf("1. curDist: %f\n",teleProps.getdist(0));
					printf("2. X: %f\n",teleProps.getX(1));
					printf("2. Y: %f\n",teleProps.getY(1));
					printf("2. T: %f\n",teleProps.getT(1));
					printf("2. FoV: %f\n",teleProps.getFoV(1));
					printf("2. minDist: %f\n",teleProps.getminD(1));
					printf("2. maxDist: %f\n",teleProps.getmaxD(1));
					printf("2. curDist: %f\n",teleProps.getdist(1));
		}
					
		else if ( msg_type == RR4COM_PROPERTY_BATTERY ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: PROPERTY Battery");
					BatteryPropertiesTelegram batteryProps = roburoc4Comm->getBatteryPropertiesTelegram();
					printf("\nBattery properties:\n");
					printf("Nominal Voltage: %f\n", batteryProps.getNominalVoltage());
					printf("Energy: %f\n", batteryProps.getEnergy());
					printf("Minimum power pct: %d\n", batteryProps.getMinimumPowerPercentage());

		}
					
		else if ( msg_type == RR4COM_PROPERTY_LOCALIZATION ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: PROPERTY Localization");
					LocalizationPropertiesTelegram localProps = roburoc4Comm->getLocalizationPropertiesTelegram();
					printf("\nLocalization properties:\n");
					printf("Initial X: %f\n",  localProps.getInitialX());
					printf("Initial Y: %f\n",  localProps.getInitialY());
					printf("Initial Theta: %f\n",  localProps.getInitialTheta());
					printf("State: %d\n",  localProps.getState());
		}
					
		else if ( msg_type == RR4COM_PROPERTY_DIFFERENTIAL ) {
					ROS_DEBUG_STREAM("Roburoc4 Driver: PROPERTY Differential");
					DifferentialPropertiesTelegram diffProperties = roburoc4Comm->getDifferentialPropertiesTelegram();
					printf("\nDifferential properties:\n");
					printf("Target Linear Speed: %f\n", diffProperties.getTargetLinearSpeed());
					printf("Target Angular Speed: %f\n", diffProperties.getTargetAngularSpeed());
					printf("Current Linear Speed: %f\n", diffProperties.getCurrentLinearSpeed());
					printf("Current Angular Speed: %f\n", diffProperties.getCurrentAngularSpeed());
					printf("Max Linear Speed: %f\n", diffProperties.getMaximumLinearSpeed());
					printf("Min Linear Speed: %f\n", diffProperties.getMinimumLinearSpeed());
					printf("Max Angular Speed: %f\n", diffProperties.getMaximumAngularSpeed());
					printf("Min Angular Speed: %f\n", diffProperties.getMinimumAngularSpeed());
					printf("Max Linear Acceleration: %f\n", diffProperties.getMaximumLinearAcceleration());
					printf("Min Linear Acceleration: %f\n", diffProperties.getMinimumLinearAcceleration());
					printf("Max Angular Acceleration: %f\n", diffProperties.getMaximumAngularAcceleration());
					printf("Min Angular Acceleration: %f\n", diffProperties.getMinimumAngularAcceleration());
					printf("Width: %f\n", diffProperties.getWidth());
					
		}
		
		else {
					ROS_DEBUG_STREAM("Roburoc4 Driver: Unknown telegram type!");
		}
		
	}
	
}

/* The velocity callback to be able to set the velocity of the robot */
void RobuROC4Driver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmdVel){
	
	ROS_DEBUG_STREAM("Roburoc4 Driver: Requesting Linear velocity:" << (float)cmdVel->linear.x << " and Angular velocity:" << (float)cmdVel->angular.z );
	
	/* Send the velocity to the robot */
	roburoc4Comm->setVelocity((float)cmdVel->linear.x / 1.3 ,(float)cmdVel->angular.z / 1.3);	// Divided by 1.3 to compensate for wheel size	
	
}

/* Request properties from a device in the roburoc */
void RobuROC4Driver::requestPropertiesCallback(const std_msgs::Int32::ConstPtr& serviceId){

	ROS_DEBUG_STREAM("Roburoc4 Driver: Request of properties for: " << serviceId->data);

	/* Request properties from the robot */
	roburoc4Comm->requestProperties(serviceId->data);	
}


void RobuROC4Driver::setDigitalOutputCallback(const std_msgs::UInt8::ConstPtr& out){

	ROS_DEBUG_STREAM("Roburoc4 Driver: Setting digital output: " << out->data);

	roburoc4Comm->setDigitalOutput(out->data);
	
}

void RobuROC4Driver::setCurrentPositionCallback(const geometry_msgs::Pose2D::ConstPtr& pos){
	
	ROS_DEBUG_STREAM("Roburoc4 Driver: Setting current Position!");
	
	roburoc4Comm->setPosition(pos->x,pos->y,pos->theta);
	
}


/* The main function of the node */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "roburoc4_driver");
	
	/* Start the roburoc4 class */
	RobuROC4Driver *roburoc4 = new RobuROC4Driver();
	
	/* Listen for callbacks */
	ros::spin();

	/* Delete the Roburoc4 */
	delete roburoc4;
	
	return 0;	
}
