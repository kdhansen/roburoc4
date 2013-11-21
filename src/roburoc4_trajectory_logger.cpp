/***********************************************************************
 * RobuROC4 trajectory logger.
 *
 * By Rune Madsen, Aalborg University, Nov. 2012
 * 
 * Log and playback trajectories for the helmsman and trajectory controllers
 * ********************************************************************/

/* ROS things */
#include <ros/ros.h>
#include <ros/console.h>

/* Messages */
#include <roburoc4/Trajectory.h>
#include <roburoc4/TrajectoryPoints.h>
#include <roburoc4/LineSegment.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>							// Messages to print string to a topic

#include <roburoc4_controllers.hpp>
#include <roburoc4/States.h>

/* File handling */
#include <fstream>
#include <boost/filesystem.hpp>

using namespace std;
namespace fs = boost::filesystem;

/* Helper functions */
void split(const string& s, char c, vector<string>& v);
string generateDate();
int nChars( std::string message, unsigned char c );

class TrajectoryLoggerNode {
	
	public:
		TrajectoryLoggerNode();
		~TrajectoryLoggerNode();
		
		void loggerLoop();
		
	private:
		/* The ros node handle */
		ros::NodeHandle nh_;
		
		/* Publishers */
		ros::Publisher lineSegmentPub_;			// Linesegments for the helmsman controller
		ros::Publisher trajectoryPub_;			// Trajectory for the trajectory controller
		ros::Publisher stringPub_;				// A string!
		
		/* Subscriber */
		ros::Subscriber stateSub_;
		ros::Subscriber loggerCmdSub_;
		
		/* Callback function for receiving the trajectory and current states */
		void statesCallback(const roburoc4::States::ConstPtr& newStates);
		void loggerCmdCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg);
		
		/* The current trajectory */
		roburoc4::Trajectory trajectory;
		
		/* Current line segment */
		roburoc4::LineSegment lineSegment;
		
		/* Current states */
		roburoc4::States states;
		
		/* Vars used to save and load trajectory files */
		string baseFolderName;
		string helmsmanFile;
		string trajectoryFile;
		
		/* Logging trajectory flag */
		bool loggingFlag;
		
};

TrajectoryLoggerNode::TrajectoryLoggerNode() {

	/* Publishers */
	lineSegmentPub_ = nh_.advertise<roburoc4::LineSegment>("roburoc4_controllers/lineSegment",1);
	trajectoryPub_ = nh_.advertise<roburoc4::Trajectory>("roburoc4_controllers/trajectory",1);
	stringPub_ =  nh_.advertise<std_msgs::String>("roburoc4_printString", 10);

	/* Subscribers */
	stateSub_ = nh_.subscribe<roburoc4::States>("roburoc4_state_estimator/states", 1 , &TrajectoryLoggerNode::statesCallback, this);
	loggerCmdSub_ = nh_.subscribe<std_msgs::UInt8>("roburoc4_controllers/logger_cmd", 1 , &TrajectoryLoggerNode::loggerCmdCallback, this);
	
	/* Different used vars */
	roburoc4::TrajectoryPoints point;
	
	/* Look for folders */
	if (nh_.getParam("/roburoc4_controllers/recorded_trajectories", baseFolderName)){
		
		/* Test if some folders exists and create them is they don't */
		
		// The base helmsman folder
		fs::path data_dir;
		fs::path data_dir2;
		
		data_dir = fs::path( baseFolderName + "/helmsman");		
		if (!fs::is_directory(fs::status(data_dir))) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No folder exists for data to the helmsman controller, creating folder!");
			fs::create_directories(data_dir);
		}

	
		// The helmsman use_this folter
		data_dir = fs::path( baseFolderName + "/helmsman-use_this");		
		if (!fs::is_directory(fs::status(data_dir))) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No folder exists to forse use a specific trajectory in the helmsman controller, creating folder!");
			fs::create_directories(data_dir);
		}

		
		// The base trajectory folder
		data_dir = fs::path( baseFolderName + "/trajectory");		
		if (!fs::is_directory(fs::status(data_dir))) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No folder exists for data to the trajecotory controller, creating folder!");
			fs::create_directories(data_dir);
		}
		
		// The trajecotry use_this folter
		data_dir = fs::path( baseFolderName + "/trajectory-use_this");		
		if (!fs::is_directory(fs::status(data_dir))) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No folder exists to forse use a specific trajectory in the trajecotry controller, creating folder!");
			fs::create_directories(data_dir);
		}


		/* Used to search the directories for the most resent file */
		fs::directory_iterator end_iter;
		fs::directory_iterator dir_iter;
		typedef std::multimap<std::time_t, std::string> result_set_t;
		result_set_t::reverse_iterator file_it;

		
		data_dir = fs::path( baseFolderName + "/helmsman-use_this");
		data_dir2 = fs::path( baseFolderName + "/helmsman");
		if ( !fs::is_empty(data_dir) ) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: Loading a file in the use_this folder");
			
			for (dir_iter = fs::directory_iterator(data_dir) ; dir_iter != end_iter ; dir_iter++) {
				if (fs::is_regular_file(dir_iter->status())){
					helmsmanFile = (dir_iter->path()).string();
					ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: File loaded for helmsman:" << helmsmanFile);
					break;
				}
			}
		} 
		else if ( !fs::is_empty(data_dir2) ) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: Loading the most resent file for the helmsman controller");
			
			result_set_t helmsman_files;
			for (dir_iter = fs::directory_iterator(data_dir2) ; dir_iter != end_iter ; dir_iter++) {
				if (fs::is_regular_file(dir_iter->status())){
					helmsman_files.insert(result_set_t::value_type(fs::last_write_time(*dir_iter), (dir_iter->path()).string()));
				}
			}
			
			file_it = helmsman_files.rend();file_it++;
			helmsmanFile = (*file_it).second;
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: File loaded for helmsman:" << helmsmanFile);		
			
			
		} else {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No data files for the helmsman controller exists");
		}
		
		data_dir = fs::path( baseFolderName + "/trajectory-use_this");
		data_dir2 = fs::path( baseFolderName + "/trajectory");
		if ( !fs::is_empty(data_dir) ) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: Loading a file in use_this folder");
			
			for (dir_iter = fs::directory_iterator(data_dir) ; dir_iter != end_iter ; dir_iter++) {
				if (fs::is_regular_file(dir_iter->status())){
					trajectoryFile = (dir_iter->path()).string();
					ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: File loaded for trajectory:" << trajectoryFile); 
					break;
				}
			}
			
		}
		else if ( !fs::is_empty(data_dir2) ) {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: Loading the most resent file for the trajectory controller");
			
			result_set_t trajectory_files;
			
			for (dir_iter = fs::directory_iterator(data_dir2) ; dir_iter != end_iter ; dir_iter++) {
				if (fs::is_regular_file(dir_iter->status())){
					trajectory_files.insert(result_set_t::value_type(fs::last_write_time(*dir_iter), (dir_iter->path()).string()));
				}
			}
			
			
			file_it = trajectory_files.rend(); file_it++;
			trajectoryFile = (*file_it).second;
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: File loaded for trajectory:" << trajectoryFile);
					
			
		} else {
			ROS_INFO_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No data files for the trajectory controller exists");
		}
		
		
	} else {
		ROS_WARN_STREAM("ROBUROC4_TRAJECTORY_LOGGER: No data folder is specified, no data will be loaded/saved");
	} 
	
	
	/* Loading data from the found files in the folders (if any file found)
	 * helmsmanFile
	 * trajectoryFile
	 */
	 
	 // temporary string used contain the lines extracted from the files
	 ifstream ifs;
	 string temp;
	 
	/** Helmsman file **/
	 if ( helmsmanFile.length() != 0 ) {
 
		ifs.open(helmsmanFile.c_str(), ifstream::in);

		/* Get the four points of interest from the file */
		ifs >> lineSegment.start.x >> lineSegment.start.y >> lineSegment.end.x >> lineSegment.end.y;

		ifs.close();

		ROS_DEBUG_STREAM("ROBUROC4_TRAJECTORY_LOGGER: Helmsman controller linesegment loaded! ");
		 
	 }
	 
	
	/** Trajectory file **/
	// Clear the list of points
	trajectory.points.clear();
	
	// If a file is found
	if ( trajectoryFile.length() != 0 ) {
		
		/* Open the file */
		ifs.open(trajectoryFile.c_str(), ifstream::in);
		vector<string> fields;

		/* Generate the file by extracting the needed information line by line */
		while ( getline(ifs,temp) ) {
			
			/* If the lines contain 4 commas */
			if ( nChars(temp,',') == 4 ) {
				
				/* Split the string by the commas */
				split(temp,',',fields);
				
				/* Extract the data from the trajectory */
				point.pose.x = atof(fields[0].c_str());
				point.pose.y = atof(fields[1].c_str());
				point.pose.theta = atof(fields[2].c_str());
				point.velocity.angular.z = atof(fields[3].c_str());
				point.velocity.linear.x = atof(fields[4].c_str());
			
				trajectory.points.push_back(point);
				
				/* Clear the fields and empty the tmp string */
				fields.clear();
				
			}
		}
		
		 ROS_DEBUG_STREAM("ROBUROC4_TRAJECTORY_LOGGER: Trajectory controller trajectory loaded!");	
		 ifs.close();	
	}			
}

TrajectoryLoggerNode::~TrajectoryLoggerNode() {

}

/* The control loop used when the controller is running */
void TrajectoryLoggerNode::loggerLoop() {

	/* Define control rate */
	ros::Rate r(20);
	
	/* The point variable to contain the point that is stored */
	roburoc4::TrajectoryPoints point;
	
	/* Do control */
	while ( ros::ok() ) {
		
		/* If logging */
		if ( loggingFlag ) {
			
			/* Generate the point on the trajectory from the logged states */
			point.pose.x = states.x;
			point.pose.y = states.y;
			point.pose.theta = states.theta;
			point.velocity.linear.x = states.v;
			point.velocity.angular.z = states.omega;
			
			/* Put in the vetor of points in the trajectory */
			trajectory.points.push_back(point);			
			
		}
		
		/* Sleep and spin to see for any new data on the topics */
		r.sleep();
		ros::spinOnce(); 
		
	}
	
}


/* Callback to update the local copy of the states */
void TrajectoryLoggerNode::statesCallback(const roburoc4::States::ConstPtr& newStates){
	
	/* Update the local copy of the states */
	states.x = newStates->x;
	states.y = newStates->y;
	states.theta = newStates->theta;
	states.omega = newStates->omega;
	states.v = newStates->v;

}


/* The callback for setting the control mode */
void TrajectoryLoggerNode::loggerCmdCallback(const std_msgs::UInt8::ConstPtr& logger_cmd_msg) {

	std_msgs::String aString;
	ofstream outputFile;
	outputFile.precision(15);
	
	roburoc4::TrajectoryPoints point;
	vector<roburoc4::TrajectoryPoints>::iterator pointsIt;


	switch ( logger_cmd_msg->data ) {
		
			/* Helmsman controller: log start point */
			case LOGGER_MODE_HELMS_START:
				lineSegment.start.x = states.x;
				lineSegment.start.y = states.y;
				
				aString.data = "Start point logged";
				stringPub_.publish(aString);
				break;
			
			/* Helmsman controller: log end point */
			case LOGGER_MODE_HELMS_END:
				lineSegment.end.x = states.x;
				lineSegment.end.y = states.y;
				
				aString.data = "End point logged";
				stringPub_.publish(aString);
				break;
			
			/* Helmsman controller: save logged points */
			case LOGGER_MODE_HELMS_SAVE:
			
				aString.data = "Helmsman trajectory could NOT be saved!";
			
				if ( baseFolderName.length() != 0 ){
					
					outputFile.open((baseFolderName + "/helmsman/" + generateDate() + ".txt").c_str());
					
					if (outputFile.is_open()){
					
						outputFile << lineSegment.start.x << endl << lineSegment.start.y << endl << lineSegment.end.x << endl << lineSegment.end.y << endl ;
						outputFile.close();
					}			
					
					aString.data = "Helmsman trajectory saved to file: " + generateDate() + ".txt";
				} 	
				stringPub_.publish(aString);
				break;
			
			/* Helmsman controller: Publish the current line segment */
			case LOGGER_MODE_HELMS_PUB:
				lineSegmentPub_.publish(lineSegment);
				
				aString.data = "Line segment published";
				stringPub_.publish(aString);
				break;
			
			/* Trajectory controller: start logging of points */
			case LOGGER_MODE_TRAJ_START:
			
				/* If asked to start logging of trajectory, and logging is disabled, clear the current trajectory and enable logging */
				if (!loggingFlag) {
					loggingFlag = true;
					
					trajectory.points.clear();
					
					aString.data = "Logging Started!";
					stringPub_.publish(aString);
				}
			
				break;
			
			/* Trajectory controller: end logging of points (including saving to file) */
			case LOGGER_MODE_TRAJ_END:
			
				/* If asked to stop logging of trajecotry, and logging is enabled, stop logging, and save the logged trajectory to a file */
				if (loggingFlag) {
					loggingFlag = false;
					
					aString.data = " Logging Ended: But file could NOT be saved! ";
					
						
					/* Find the last point of interest in the trajectory */
					for ( pointsIt = trajectory.points.end() ; pointsIt != trajectory.points.begin() ; --pointsIt ) {
						if ( abs((*pointsIt).velocity.angular.z) > 0.01 && abs((*pointsIt).velocity.linear.x) > 0.01 ) 
							break; 
					}
					
					/* Delete the points after the last point of interest */
					trajectory.points.erase(pointsIt,trajectory.points.end());
					
					/* Find the first point of interest in the trajectory */
					for ( pointsIt = trajectory.points.begin() ; pointsIt != trajectory.points.end() ; pointsIt++ ) {
						if ( abs((*pointsIt).velocity.angular.z) > 0.01 && abs((*pointsIt).velocity.linear.x) > 0.01 ) 
							break; 
					}
					
					/* Delete the points before the first point of interest */
					trajectory.points.erase(trajectory.points.begin(),pointsIt);
					
					/* If any points are logged and a folder is specifies, save the trajectory */
					if (!trajectory.points.empty() && baseFolderName.length() != 0){
					
						outputFile.open((baseFolderName + "/trajectory/" + generateDate() + ".txt").c_str());
						
						if (outputFile.is_open()){
							
								for ( pointsIt = trajectory.points.begin() ; pointsIt != trajectory.points.end() ; pointsIt++ ) {
									outputFile << (*pointsIt).pose.x << "," << (*pointsIt).pose.y << "," << (*pointsIt).pose.theta << "," << (*pointsIt).velocity.angular.z << "," << (*pointsIt).velocity.linear.x << endl;
								}
								
								/* Save a point with completely zero velocity as the last point */
								pointsIt--;
								outputFile << (*pointsIt).pose.x << "," << (*pointsIt).pose.y << "," << (*pointsIt).pose.theta << "," << 0 << "," << 0 << endl;					
												
								outputFile.close();
								
								aString.data = "Logging Ended: saved to file: " + generateDate() + ".txt";
						}
						
					} else if (trajectory.points.empty()) {
						aString.data = "No driving data logged!";
					}
					
					stringPub_.publish(aString);
				}
			
				break;
			
			/* Trajectory controller: publish the logged points */
			case LOGGER_MODE_TRAJ_PUB:
			
				trajectoryPub_.publish(trajectory);
				
				aString.data = "Trajectory Published";
				stringPub_.publish(aString);
			
				break;
				
			default:
				break;	
		
	}	
}


/* The main function starts the controller */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "roburoc4_trajectory_logger");
	TrajectoryLoggerNode loggerNode;

	loggerNode.loggerLoop();

	return 1;
}



/** Helper functions **/
/* Function to split a string at a given char */
void split(const string& s, char c, vector<string>& v) {
   string::size_type i = 0;
   string::size_type j = s.find(c);
   while (j != string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);
      if (j == string::npos)
         v.push_back(s.substr(i, s.length( )));
   }
}

/* Generate the current date in format: YYYY_MM_DD_HH_II */
string generateDate() {
	
	time_t rawtime;
	struct tm * timeinfo;

	/* Generate the time variables */
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	
	/* Format the time */
	char date[300];
	sprintf(date,"%04d_%02d_%02d_%02d_%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min);
 	
 	/* Generate the string from the formated char array, and return it */
 	return string(date);
 		
}

/* Count number of a given char in a string */
int nChars( std::string message, unsigned char c ) {
	
	int nChar = 0;
	
	for (unsigned int i = 0; i < message.length() ; i++) {
		
		if (message.at(i) == c)
			nChar++;	
	}
	
	return nChar;
}
