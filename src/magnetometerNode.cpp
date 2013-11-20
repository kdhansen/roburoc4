#include <ros/ros.h>  				// Ros declerations
#include <ros/console.h>			// DEBUG COMMANDS

#include <geometry_msgs/Pose2D.h>	// Message to contain orientation to be published
#include <std_msgs/Empty.h>			// Empty message
#include <std_msgs/String.h>		// String message

#include <fstream>					// File IO libary

#include <armadillo>				// Armadillo linear algebra libary
#include "helper_functions/gnuplot_i.hpp"			// GNUplot c API libary
#include "helper_functions/serialCommunicator.hpp" 	// Serial communication
#include "helper_functions/helper_functions.hpp"		// Helper functions for splitting a string etc.

#include <string>					// std::string


using namespace std;
using namespace arma;

class MagnetometerNode {
	
	public:
	/* Constructor */
	MagnetometerNode();
	~MagnetometerNode();
		
	/* Main loop function */
	void loop();
	
	private:
	/* ROS fields */
	ros::NodeHandle nh_; 				// Node handle
	
	ros::Subscriber calibModeSub_;
	
	ros::Publisher orientationPub_;		// Publisher for the orientation
	ros::Publisher stringPub_;			// Publisher to putlish to the string topic
	
	/* Serial Communication fields */
	serialCommunicator* serialCom_;		// Serial communicator
	string serialPort_;					// The serial port name where the magnetometer is connected
	int serialBaud_;					// The serial port baud rate	

	/* calibration mode */
	int calibMode_;
	
	/* calibration parameters */
	double xOffset, yOffset, xScale, yScale;
	
	/* Degauss removal variables */
	double magXOld, magYOld, xOff, yOff, degaussThr;
	int xFlag, yFlag, xCnt, yCnt;

	/* Calibration file andle and filename */
	string calibrationFileFilename;
	fstream magCaliFile;
	
	/* Callback for the calibration mode setter */
	void calibModeCallback(const std_msgs::Empty::ConstPtr& empty);	
};

MagnetometerNode::MagnetometerNode():
	serialPort_("/dev/ttyUSB2"),
	serialBaud_(19200)
 {
	
	/* Get the parameters from the parameter server */
	nh_.param<std::string>("/MagnetometerNode/serial_port_name",serialPort_,serialPort_);
	nh_.param("/MagnetometerNode/serial_port_baud",serialBaud_,serialBaud_);
	
	/* Disp debug information About the serial port*/
	ROS_INFO_STREAM("MAGNETOMETER NODE: Serial port: " << serialPort_);
	ROS_INFO_STREAM("MAGNETOMETER NODE: Baud rate: " << serialBaud_);
	
	/* Connect to the serial communicator */
	try{
		serialCom_ = new serialCommunicator(serialPort_,serialBaud_);
	} catch (std::string message) {
		ROS_FATAL_STREAM( "MAGNETOMETER NODE: Error creating the serialCommunicator: " << message );
		exit(0);	// Close the node!
	}
		
	/* Setup the magnetometer to print out the correct format*/
	// TODO: Make this setup
	
	/* Publisher: */
	orientationPub_ = nh_.advertise<geometry_msgs::Pose2D>("Magnetometer/orientation", 10);
	stringPub_ =  nh_.advertise<std_msgs::String>("roburoc4_printString", 10);
	
	/* Subscriber:  Get signal to calibrate the sensor */
	calibModeSub_ = nh_.subscribe<std_msgs::Empty>("Magnetometer/calibrate", 1, &MagnetometerNode::calibModeCallback, this);
	
	/* Generate the calibration file name */
	calibrationFileFilename = "magnetometer_calibration_file.txt";
	
	/* Setup the default calibration mode */
	calibMode_ = 0;
	
	/* Load calibration scaling factors or use default */
	/* Open handle to the magnetometer calibration file */
	magCaliFile.open(calibrationFileFilename.c_str());
	if (magCaliFile.is_open()) {
	
		/* Load old coefficients (0 if file is empty)*/
		magCaliFile >> xOffset >> yOffset >> xScale >> yScale;
		magCaliFile.close();
		ROS_INFO_STREAM("MAGNETOMETER NODE: Calibration loaded: xO = " << xOffset << " yO = " << yOffset << " xS = " << xScale << " yS = " << yScale );
	
	} else{
		
		xScale = 1.130774550539680 / 100.0;
		yScale = 0.917835058551920 / 100.0;
		xOffset = 9.448823526767640;
		yOffset = 15.162688940048779;
		
		ROS_INFO_STREAM("MAGNETOMETER NODE: Calibration file could not be opened, using default calibration: xO = " << xOffset << " yO = " << yOffset << " xS = " << xScale << " yS = " << yScale );
	}
	
	/* Setup default values for the magnetometer cdegauss */
	degaussThr = 6;
	magXOld = 0;
	magYOld = 0;
	xCnt = 0;
	yCnt = 0;
	xFlag = 0;
	yFlag = 0;
	
	/* Disp debug information that the node has started*/
	ROS_INFO("MAGNETOMETER NODE: MagnetometerNode started");
	
}

MagnetometerNode::~MagnetometerNode() {
	
	/* Delete the serial communicator */
	delete serialCom_;
	
}

void MagnetometerNode::loop(){
		
	/* Allocate variables and matrices for ellipse fitting */
	int numPts = 100	; // Define number of points that each fitting should be defined by
	int cnt = 0;
	
	/* Variables for the calibration (least squares ellipse fitting) */
	double magXo = 10000;
	double magYo = 10000;
	double c;
	
	double xPlot[numPts], yPlot[numPts];
	
	double xOffsetNew, yOffsetNew, xScaleNew, yScaleNew;
	
	mat A(numPts,5);
    mat pts(2,numPts);
    mat q(3,3);
    mat centm(2,2);
    mat eigvec(2,2);
    
    colvec b(numPts); b.ones();
    colvec p(5);
    colvec v(2);
    colvec centre2(2);
    colvec centre3(3);
    colvec eigval(2);
    
	/* While ros is okay */
	while (ros::ok()) {
	
		/* Get a single line from the serial port */
		string serialMessage = serialCom_->getLine();
		
		/* Do the received string contain any data */
		if ( serialMessage.length() > 0 ) {
						
			/* Let $ be the first char in the string */
			if (serialMessage.find("$") < serialMessage.length()) {
				serialMessage = serialMessage.substr(serialMessage.find("$"));
			}
			
			/* Check if the received string has the correct format (checksum check) */
			//TODO: CHECKSUM DOES NOT WORK HERE!! , dont know why?
			//if ( calculateNMEAChecksum(serialMessage) !=  serialMessage.substr(serialMessage.find("*")+1,2) ) {	// If not, write an error message

			if (serialMessage.find("$OHPR") > serialMessage.length()) {
				ROS_WARN_STREAM("MAGNETOMETER NODE: Checksum: (" << calculateNMEAChecksum(serialMessage) << ") EXPECTED: (" << serialMessage.substr(serialMessage.find("*")+1,2) << ")");
				
			} else {
				
				
				/* Remove the $OPHR heading and the ,*<checksum> tail */
				serialMessage = serialMessage.substr(serialMessage.find(' '));
				serialMessage = serialMessage.substr(0,serialMessage.rfind('*')-1);
				
				/* Split the string by the commas */
				vector<string> fields;
				split(serialMessage,',',fields);
				
				if ( fields.size() != 7 ) {
					ROS_WARN_STREAM("MAGNETOMETER NODE: Wrong number of fields in string: " << fields.size() << " (7)");
				} else {
					
					/***
					* If the messages enters here it has the following format
					* $OHPR 353.8,-42.43,98.58,-11514.75,0.027,-0.005,-1.034,*3F
					* $OHPR head, Mx, My, Mz, Gx, Gy, Gz, checkSum
					***/
								
					/* Extract the calues from the string */
					double head = atof(fields[0].c_str());
					double magX = atof(fields[1].c_str());
					double magY = atof(fields[2].c_str());
					//~ double magZ = atof(fields[3].c_str());
					//~ double accX = atof(fields[4].c_str());
					//~ double accY = atof(fields[5].c_str());
					//~ double accZ = atof(fields[6].c_str());
			
					/* Remove the degauss from the data */
						// From x-sensor
						// Detect degauss
						if ( xCnt == 0 && xFlag == 0) {
							xOff = magX - magXOld;
							
							if ( abs(xOff) > degaussThr ) {
								xCnt = 4;
							}
						}
						// Compensate for degauss
						xFlag = 0;
						if ( xCnt > 0 ) {
							magX = magX - xOff;
							xCnt--;
							xFlag = 1;
						}
						
						// From y-sensor
						// Detect degauss
						if ( yCnt == 0 && yFlag == 0) {
							yOff = magY - magYOld;
							
							if ( abs(yOff) > degaussThr ) {
								yCnt = 4;
							}
						}
						// Compensate for degauss
						yFlag = 0;
						if ( yCnt > 0 ) {
							magY = magY - yOff;
							yCnt--;
							yFlag = 1;
						}
						
						// Save the current as the old data
						magXOld = magX;
						magYOld = magY;

					
					/* If not in calibration mode, extract the heading and publish the calculated orientation */
					if ( calibMode_ == 0 ) {
					
						/* Calculate the heading from the string */
						geometry_msgs::Pose2D orientation;
						
						/* Save the orientation calculated by the magnetometer as an orientaion around the x-axiz */
						orientation.x = head;
						
						/* Do the "calibration" of the inputData */
						magY = (magY - yOffset) * yScale;
						magX = (magX - xOffset) * xScale;
						
						/* Save the orientation calculated by the raw magnetic field output */
						orientation.theta = atan( magY / magX);
						
						if ( magX < 0 ){
							orientation.theta = orientation.theta + M_PI ;	
						}
						else if( (magX > 0 ) && ( magY < 0) ){
							orientation.theta = orientation.theta + 2*M_PI;
						}
						
						/* Convert the data into the desired coordinate system (And Add PI to compensate for wrong direction :-) , and subtract 0.05 to compensate for a small rotation) */
						orientation.theta = fmod( M_PI_2 - orientation.theta + 2 * M_PI + M_PI - 0.05, 2*M_PI);
						
						/* Publish the heading to a topic */
						orientationPub_.publish(orientation);
					
					}
					/* Otherwise capture the desired data and find new offset and scaling factor */
					else{
						
						/* Print info in the beginning of each calibration */
						if (cnt == 0) {
							ROS_INFO("MAGNETOMETER NODE: Calibration started. \n Start to drive robot in circles");
							ROS_INFO_STREAM("MAGNETOMETER NODE: Old Params: xO = " << xOffset << " yO = " << yOffset << " xS = " << xScale << " yS = " << yScale );
						}
						
						/* Check if new data is within a radius of [50,150] of centre and different from the previous data */
						if (sqrt(pow(magX,2) + pow(magY,2)) > 50 && sqrt(pow(magX,2) + pow(magY,2)) < 150 && sqrt(pow(magX-magXo,2)+pow(magY-magYo,2)) > 628*2/numPts) {
							
							cout << cnt << flush; // Print how much valid magnetometer data has been collected
							
							/* Save the new valid magetometer data*/
							A(cnt,0) = pow(magX,2);
							A(cnt,1) = magX*magY;
							A(cnt,2) = pow(magY,2);
							A(cnt,3) = magX;
							A(cnt,4) = magY;
							
							cnt++; // Indicate new data has been saved
							
							/* Save old magnetometer data */
							magXo = magX;
							magYo = magY;
							
							/* If enough data has been collected calculate magnetometer calibration coefficients by LS ellipse fitting*/
							if (cnt == numPts) {
								
								p = pinv(A)*b; // Calculate the coefficients of the conic section using least square
								
								/* Define the quadratic form of the conic section */
								q	<< p(0)     << p(1)/2   << p(3)/2 << endr
									<< p(1)/2   << p(2)     << p(4)/2 << endr
									<< p(3)/2   << p(4)/2   << -1     << endr;
								
								/* Define matrix for centre calculation */
								centm   << p(0)     << p(1)/2    << endr
										<< p(1)/2   << p(2)    << endr;
								
								v	<< -p(3)/2  << -p(4)/2; // Extract D and E coefficients
								
								centre2 = centm.i()*v; // Find centre of the ellipse
								
								/* Insert new offset coefficents */
								xOffsetNew = as_scalar(centre2(0));
								yOffsetNew = as_scalar(centre2(1));
								
								/* Scale the centm matrix with the coefficient c */
								centre3 << centre2(0) << centre2(1) << 1;
								c = as_scalar(centre3.t()*q*centre3);
		
								centm   << p(0)/-c       << (p(1)/2)/-c	<< endr
										<< (p(1)/2)/-c   << p(2)/-c    	<< endr;
											
								/* Calculate the eigenvalues and vectors of the centm matrix */
								eig_sym(eigval, eigvec, centm);
								
								/* Determine if the major axis points mostly in the x or y direction */
								if (abs(atan( eigvec(1,0) / eigvec(0,0) )) > 0.7854) {
									/* Use the length of the major and minor axis of the ellipse as scaling factors */
									xScaleNew = 1/sqrt(1/eigval(1));
									yScaleNew = 1/sqrt(1/eigval(0));
								} else {
									/* Use the length of the major and minor axis of the ellipse as scaling factors */
									xScaleNew = 1/sqrt(1/eigval(0));
									yScaleNew = 1/sqrt(1/eigval(1));
								}
								
								/* Extract the calibrated magnetometer data for plotting */
								for (int i = 0; i < numPts; i ++) {
									xPlot[i] = (as_scalar(A(i,3))-xOffsetNew) * xScaleNew;
									yPlot[i] = (as_scalar(A(i,4))-yOffsetNew) * yScaleNew;
								}
								
								/* Get handle to GNUplot */
								char ok;
								gnuplot_ctrl * h1; 
								h1 = gnuplot_init();
								gnuplot_resetplot(h1) ;
								gnuplot_setstyle(h1, "lines");
								gnuplot_plot_xy(h1, xPlot, yPlot, numPts, "Calibrated Magnetometer data") ;
								gnuplot_cmd(h1,"set xrange [-1.5:1.5]" );
								gnuplot_cmd(h1,"set yrange [-1.5:1.5]");
								gnuplot_cmd(h1,"set size square");
								
								/* Let user decide if calibration is acceptable */
								ROS_INFO("MAGNETOMETER NODE: Is calibration OK? y/n/a");
								cin >> ok;
								
								if (ok == 'y') {
									calibMode_ = 0;					
									
									/* Write the calibration to file */
									magCaliFile.open( calibrationFileFilename.c_str() );
									
									if (magCaliFile.is_open()) {
										magCaliFile << xOffsetNew << endl << yOffsetNew << endl << xScaleNew << endl << yScaleNew;
										magCaliFile.close();
										xOffset = xOffsetNew;
										yOffset = yOffsetNew;
										xScale = xScaleNew;
										yScale = yScaleNew;
									} else{
										ROS_INFO("Calibration file could not be opened!");
									}
									
									ROS_INFO("MAGNETOMETER NODE: Calibration completed");
									ROS_INFO_STREAM("MAGNETOMETER NODE: New Params: xO = " << xOffset << " yO = " << yOffset << " xS = " << xScale << " yS = " << yScale );
								
								} else if (ok == 'n') {
								} else {
									ROS_INFO_STREAM("MAGNETOMETER NODE: Calibration aborted, using previous calibration: xO = " << xOffset << " yO = " << yOffset << " xS = " << xScale << " yS = " << yScale );							
								}
								
								gnuplot_close(h1) ;
								
								/* Reset calibration coefficients */
								cnt = 0;
								magXo = 10000;
								magYo = 10000;
							}
						} else {
							cout << "." << flush;
						}
					}
				}					
			}
		}
		ros::spinOnce();
	}
}

void MagnetometerNode::calibModeCallback(const std_msgs::Empty::ConstPtr& empty){
	
	std_msgs::String aString;	
	
	if ( calibMode_ == 0 ) {
		calibMode_ = 1;
		ROS_INFO_STREAM("MAGNETOMETER NODE: Entering calibration mode");
		aString.data = "Magnetometer entered calibration mode";
		
	} else {
		calibMode_ = 0;
		aString.data = "Magnetometer left calibration mode";
	}
	
	/* Publish filter reset message to a topic */
	stringPub_.publish(aString);
	
}

/* Main function */
int main(int argc, char *argv[]){
	
	/* Initialise node */
	ros::init(argc, argv, "MagnetometerNode");

	/* Generate the node object */
	MagnetometerNode magnetometerNode;	
	
	magnetometerNode.loop();	// Tale a loop
	return 0;
}
