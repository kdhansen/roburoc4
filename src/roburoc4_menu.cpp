/* ROS includes */
#include <ros/ros.h>
#include <ros/console.h>

#include <cstdlib>
#include <cstring>

/* Standard Messages */
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

/* Sensor messages */
#include <sensor_msgs/NavSatFix.h>				// Message types for the GPS position output
#include <sensor_msgs/NavSatStatus.h>

#include <geometry_msgs/Twist.h>				// Message type to contain velocities
#include <geometry_msgs/Pose2D.h>				// Message type to contain heading

#include <roburoc4/States.h>	// Message type to contain the states of the system



/* The menu ... */
#include <ncurses.h>
#include <curses.h>
#include <menu.h>

/* Defines for the controllers including the logger */
#include <roburoc4_controllers.hpp>

#include <boost/thread.hpp>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define CTRLD   4 
#define ACTIVE_TIME 20


char *choices[] = {
                        "--------- Controllers ---------",
						"Controllers OFF                ",
						"Helmsman    ON                 ",
						"Trajectory  ON                 ",
						"----------- Capture -----------",
                        "Helm: Capture Start Waypoint   ",
                        "Helm: Capture End Waypoint     ",
                        "Helm: Save Waypoints           ",
                        "Traj: Capture Trajectory       ",
						"Traj: Stop Capture Trajectory  ",
                        "----------- Publish -----------",
                        "Helm: Publish to helmsman ctrl ",
                        "Traj: Publish to traject ctrl  ",
                        "----------- Sensors -----------",
                        "Calibrate Magnetometer         ",
                        "Reset Kalman Filter            ",
                        (char *)NULL,
                  };


void print_in_middle(WINDOW *win, int starty, int startx, int width, char *string, chtype color);
void printOnlineStatus(WINDOW *win, int starty, int startx, bool status);


using namespace std;

class MenuNode {
	
	public:
		MenuNode();
		~MenuNode();
								
	private:
		/* The ros node handle */
		ros::NodeHandle nh_;
		
		/* Publishers */
		ros::Publisher controlModePub_;
		ros::Publisher calibrateMagnetometerPub_;
		ros::Publisher resetFilterPub_;
		ros::Publisher trajLoggerCmdPub_;
		
		/* Subscribers */
		ros::Subscriber controlModeSub_;
		ros::Subscriber stringPrintSub_;
	
		ros::Subscriber gpsPosSub_;				// GPS Position
		ros::Subscriber gpsVelSub_;				// GPS Linear Velocity
		ros::Subscriber gpsHeadSub_;			// GPS Heading
		ros::Subscriber gyroVelSub_;			// Gyro Angular Velocity
		ros::Subscriber magHeadSub_;			// Magnetometer Heading
		ros::Subscriber roc4VelSub_;			// RobuROC4 Linear and Angular Velocity
		ros::Subscriber stateSub_;				// The estimated states
		
		/* Callback functions */
		void controlModeCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg);
		void stringPrintCallback(const std_msgs::String::ConstPtr& string_msg);
		
		/* .... for sensor inputs */
		void gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& pos );
		void gpsVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel );
		void gpsHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head );
		void gyroVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel );
		void magnetometerHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head );
		void roc4VelocityCallback(const geometry_msgs::Twist::ConstPtr& vel );
		
		void statesCallback(const roburoc4::States::ConstPtr& newStates);
		
		/* Function to update the window on new information */
		void updateStatusWindow();


		/* Functions for the threads */
		void activeTimer();
		void catchUserInput();

		/* Thread to handle the user inputs*/
		boost::thread* uiListener;
		boost::thread* activeTimerThread;
		
		/* Timers to see if active */
		int roburoc4Timer;
		int gpsTimer;
		int magTimer;
		int gyroTimer;
		int stateEstTimer;
		
		/* Containers for menu content */
		sensor_msgs::NavSatFix gpsPos_;
		geometry_msgs::Twist gpsVel_;
		geometry_msgs::Pose2D gpsHead_;
		geometry_msgs::Twist gyroVel_;
		geometry_msgs::Pose2D magHead_;
		geometry_msgs::Twist roc4Vel_;
		roburoc4::States states_;

		/* Menu things */
		ITEM **my_items;
        int c;                          
        MENU *my_menu;
        WINDOW *my_menu_win;
        WINDOW *my_status_win;
        int n_choices, i;
        
        int menuWindowWidth, statusWindowWidth;
        
        /* Messages to contain the data from the tpoics */
        std_msgs::UInt8 controlMode;
        			
};

MenuNode::MenuNode() {
		
		/* Reset timers */
		roburoc4Timer = 0;
		gpsTimer = 0;
		magTimer = 0;
		gyroTimer = 0;
		stateEstTimer = 0;


		/* Setup the publishers publisher */
		controlModePub_ = nh_.advertise<std_msgs::UInt8>("roburoc4_controllers/controlMode", 1);
		calibrateMagnetometerPub_ = nh_.advertise<std_msgs::Empty>("Magnetometer/calibrate", 1);			
		resetFilterPub_ = nh_.advertise<std_msgs::Empty>("roburoc4_state_estimator/reset_filter", 1);
		trajLoggerCmdPub_ = nh_.advertise<std_msgs::UInt8>("roburoc4_controllers/logger_cmd", 1);
		
		/* Setup the Subscribers */
		controlModeSub_ = nh_.subscribe<std_msgs::UInt8>("roburoc4_controllers/controlMode", 1 , &MenuNode::controlModeCallback, this);
		stringPrintSub_ = nh_.subscribe<std_msgs::String>("roburoc4_printString", 1 , &MenuNode::stringPrintCallback, this);
		
		stateSub_ = nh_.subscribe<roburoc4::States>("roburoc4_state_estimator/states", 1 , &MenuNode::statesCallback, this);
		gpsPosSub_ = nh_.subscribe<sensor_msgs::NavSatFix>("GPS/position", 1, &MenuNode::gpsPositionCallback, this);
		gpsVelSub_ = nh_.subscribe<geometry_msgs::Twist>("GPS/velocity", 1, &MenuNode::gpsVelocityCallback, this);
		gpsHeadSub_ = nh_.subscribe<geometry_msgs::Pose2D>("GPS/orientation", 1, &MenuNode::gpsHeadingCallback, this);
		gyroVelSub_ = nh_.subscribe<geometry_msgs::Twist>("Gyro/velocity", 1, &MenuNode::gyroVelocityCallback, this);
		magHeadSub_ = nh_.subscribe<geometry_msgs::Pose2D>("Magnetometer/orientation", 1, &MenuNode::magnetometerHeadingCallback, this);
		roc4VelSub_ = nh_.subscribe<geometry_msgs::Twist>("roburoc4_driver/cur_vel", 1, &MenuNode::roc4VelocityCallback, this);
		
		
		/* Initialize curses */
        initscr();
        start_color();
        cbreak();
        noecho();
        keypad(stdscr, TRUE);
        init_pair(1, COLOR_GREEN, COLOR_BLACK);
        init_pair(2, COLOR_RED, COLOR_BLACK);

        /* Create items */
        n_choices = ARRAY_SIZE(choices);
        my_items = (ITEM **)calloc(n_choices, sizeof(ITEM *));
        for(i = 0; i < n_choices; ++i)
                my_items[i] = new_item(choices[i], "");

        /* Crate menu */
        my_menu = new_menu((ITEM **)my_items);

        /* Create the window to be associated with the menu */
        menuWindowWidth = 40;
	    my_menu_win = newwin(LINES-2, menuWindowWidth, 1, 2);
	    
		statusWindowWidth = COLS/3;
		if (statusWindowWidth < 40)
			statusWindowWidth = COLS - menuWindowWidth - 4;
	   
        my_status_win = newwin(LINES-2, statusWindowWidth , 1, menuWindowWidth + 2 );
        keypad(my_menu_win, TRUE);
     
        /* Set main window and sub window */
        set_menu_win(my_menu, my_menu_win);
        set_menu_sub(my_menu, derwin(my_menu_win, n_choices+2, 38, 3, 1));
        set_menu_format(my_menu,20,1);

        /* Set menu mark to the string " * " */
        set_menu_mark(my_menu, " * ");
        
        /* Make the cursor invisible */
        curs_set(0);

        /* Print a border around the main window and print a title */
        box(my_menu_win, 0, 0);
        print_in_middle(my_menu_win, 1, 0, 40, "RobuROC4 Menu", COLOR_PAIR(1));
        mvwaddch(my_menu_win, 2, 0, ACS_LTEE);
        mvwhline(my_menu_win, 2, 1, ACS_HLINE, 38);
        mvwaddch(my_menu_win, 2, 39, ACS_RTEE);
        
               
        /* Print a box to contain all Status information from the robot */
        box(my_status_win, 0, 0);
        print_in_middle(my_status_win, 1, 0, statusWindowWidth, "RobuROC4 Status", COLOR_PAIR(1));
        mvwaddch(my_status_win, 2, 0, ACS_LTEE);
        mvwhline(my_status_win, 2, 1, ACS_HLINE, statusWindowWidth-2);
        mvwaddch(my_status_win, 2, statusWindowWidth-1, ACS_RTEE);   
        
        refresh();
        
        /* Post the menu */
        post_menu(my_menu);
        
        wrefresh(my_status_win);
        wrefresh(my_menu_win);

		updateStatusWindow();
				
		/* Startup the thread for listening to the user */
		uiListener = new boost::thread( boost::bind( &MenuNode::catchUserInput, this ) );
		activeTimerThread = new boost::thread( boost::bind( &MenuNode::activeTimer, this ) );
	
	
}

MenuNode::~MenuNode() {

		/* Unpost and free all the memory taken up */
        unpost_menu(my_menu);
        free_menu(my_menu);
        for(i = 0; i < n_choices; ++i)
                free_item(my_items[i]);
        endwin();
        
        delete uiListener;
        delete activeTimerThread;
}



void MenuNode::catchUserInput() {
		
		/* Messages used to send data */
		std_msgs::UInt8 ctrlMode;
		std_msgs::UInt8 loggerCmd;
		std_msgs::Empty dummyEmpty;
	
		while( ros::ok() && (c = wgetch(my_menu_win)) != 0)
        {       switch(c)
                {       case KEY_DOWN:
                                menu_driver(my_menu, REQ_DOWN_ITEM);
                                break;
                        case KEY_UP:
                                menu_driver(my_menu, REQ_UP_ITEM);
                                break;
						case 10: /* Enter */
								if ( current_item(my_menu) == my_items[1] ) {
									
									/* Controllers OFF*/
									ctrlMode.data = CONTROL_MODE_OFF;
									controlModePub_.publish(ctrlMode);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));

									
								} else if ( current_item(my_menu) == my_items[2] ){
									
									/* Helmsman ON */
									ctrlMode.data = CONTROL_MODE_HELMSMAN;
									controlModePub_.publish(ctrlMode);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
									
								} else if ( current_item(my_menu) == my_items[3] ){
									
									/* Trajectory ON */
									ctrlMode.data = CONTROL_MODE_TRAJECTORY;
									controlModePub_.publish(ctrlMode);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));

									
								} else if ( current_item(my_menu) == my_items[5] ){
									
									/* Helmsman - Capture Start Point */
									loggerCmd.data = LOGGER_MODE_HELMS_START;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));

									
								} else if ( current_item(my_menu) == my_items[6] ){
									
									/* Helmsman - Capture End Point */
									loggerCmd.data = LOGGER_MODE_HELMS_END;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[7] ){
									
									/* Trajectory - Capture Trajectory */
									loggerCmd.data = LOGGER_MODE_HELMS_SAVE;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[8] ){
									
									/* Trajectory - Capture Trajectory */
									loggerCmd.data = LOGGER_MODE_TRAJ_START;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[9] ){
									
									/* Trajectory - STOP Capture Trajectory */
									loggerCmd.data = LOGGER_MODE_TRAJ_END;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[11] ){
									
									/* Publish Helmsman waypoints */
									loggerCmd.data = LOGGER_MODE_HELMS_PUB;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[12] ){
									
									/* Publish Trajectory for trajecotry follower controller */
									loggerCmd.data = LOGGER_MODE_TRAJ_PUB;
									trajLoggerCmdPub_.publish(loggerCmd);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[14] ){
									
									/* Calibrate Magnetometer message */
									calibrateMagnetometerPub_.publish(dummyEmpty);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else if ( current_item(my_menu) == my_items[15] ){
																	
									/* Reset Kalman filter message */
									resetFilterPub_.publish(dummyEmpty);
									mvprintw(LINES - 1, 2, "Item selected is : %s", item_name(current_item(my_menu)));
	
									
								} else  {
									
									/* Clear the "selection" line */
									mvprintw(LINES - 1, 2, "                                                                         ");
									
								} 
												 
								refresh();
								pos_menu_cursor(my_menu);								
								break;
                }
                wrefresh(my_menu_win);
        }
}

void MenuNode::activeTimer(){
	
	ros::Rate r(20);
	
	while(ros::ok() ){
	
		if (roburoc4Timer > 0)
			roburoc4Timer--;
			
		if (gpsTimer > 0)
			gpsTimer--;
			
		if (magTimer > 0)
			magTimer--;
			
		if (gyroTimer > 0)
			gyroTimer--;
			
		if (stateEstTimer > 0)
			stateEstTimer--;	
	
		/* Update the status window */
		updateStatusWindow();
	
		r.sleep();
	}
	
}

/* The main function starts the controller */
int main(int argc, char** argv)
{

	ros::init(argc, argv, "roburoc4_menu");
	MenuNode menuNode;

	/* Listen on the topics */
	ros::spin();
	
	return 1;
}

/* Function to print hte menu window */
void print_in_middle(WINDOW *win, int starty, int startx, int width, char *string, chtype color)
{       int length, x, y;
        float temp;

        if(win == NULL)
                win = stdscr;
        getyx(win, y, x);
        if(startx != 0)
                x = startx;
        if(starty != 0)
                y = starty;
        if(width == 0)
                width = 80;

        length = strlen(string);
        temp = (width - length)/ 2;
        x = startx + (int)temp;
        wattron(win, color);
        mvwprintw(win, y, x, "%s", string);
        wattroff(win, color);
        refresh();
}

void MenuNode::updateStatusWindow(){
	
	int x = 2;
	int y = 3;
	
	mvwprintw(my_status_win, y, x, "%s", "Sensors:");x++;y++;
	
	mvwprintw(my_status_win, y, x, "%s", "RobuROC4"); printOnlineStatus(my_status_win,y,x+15, roburoc4Timer > 0 ); y++; x++;
	mvwprintw(my_status_win, y, x, "%s", "Linear Velocity:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", roc4Vel_.linear.x,"m/s"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Angular Velocity:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", roc4Vel_.angular.z,"rad/s"); y++;
	
	x--;y++;
	mvwprintw(my_status_win, y, x, "%s", "GPS"); printOnlineStatus(my_status_win,y,x+15, gpsTimer > 0 ); y++; x++;
	mvwprintw(my_status_win, y, x, "%s", "Longitude:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s",gpsPos_.longitude ,"deg"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Latitude:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", gpsPos_.latitude,"deg"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Linear Velocity:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", gpsVel_.linear.x,"m/s"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Heading:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", gpsHead_.theta, "rad"); y++;
	mvwprintw(my_status_win, y, x, "%s", "N sats:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8d %s", gpsPos_.status.service, ""); y++;
	mvwprintw(my_status_win, y, x, "%s", "Status:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8d %s",  gpsPos_.status.status, ""); y++;
	
	x--;y++;
	mvwprintw(my_status_win, y, x, "%s", "Magnetometer"); printOnlineStatus(my_status_win,y,x+15, magTimer > 0); y++; x++;
	mvwprintw(my_status_win, y, x, "%s", "Heading:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", magHead_.theta, "rad"); y++;
	
	x--;y++;
	mvwprintw(my_status_win, y, x, "%s", "Gyro"); printOnlineStatus(my_status_win,y,x+15, gyroTimer > 0); y++; x++;
	mvwprintw(my_status_win, y, x, "%s", "Angular Velocity:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", gyroVel_.angular.z,"rad/s"); y++;

	x--;x--;y++;
	mvwprintw(my_status_win, y, x, "%s", "State Estimator");printOnlineStatus(my_status_win,y,x+16, stateEstTimer > 0);x++;y++;
	mvwprintw(my_status_win, y, x, "%s", "X Pos:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", states_.x,"m"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Y Pos:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", states_.y,"m"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Heading:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", states_.theta, "deg"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Linear Velocity:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", states_.v,"m/s"); y++;
	mvwprintw(my_status_win, y, x, "%s", "Angular Velocity:"); mvwprintw(my_status_win, y, statusWindowWidth -17, "%8.5f %s", states_.omega,"rad/s"); y++;
	
	x--;y++;
	mvwprintw(my_status_win, y, x, "%s", "Controllers:");x++;y++;
	mvwprintw(my_status_win, y, x, "%s", "Helmsman:");printOnlineStatus(my_status_win,y,x+15,controlMode.data == CONTROL_MODE_HELMSMAN ); y++;;
	mvwprintw(my_status_win, y, x, "%s", "Trajecotry:");printOnlineStatus(my_status_win,y,x+15,controlMode.data == CONTROL_MODE_TRAJECTORY ); y++;
		
	wrefresh(my_status_win);
		
}

void printOnlineStatus(WINDOW *win, int starty, int startx, bool status) {
	
	int color = 1;
	
	if (!status)
		color = 2;
	
	wattron(win, COLOR_PAIR(color));
	if (status)
		mvwprintw(win, starty, startx, "%s", "(online) ");
	else
		mvwprintw(win, starty, startx, "%s", "(offline)");
	wattroff(win, COLOR_PAIR(color));
	
	
}


/* Callbacks */
void MenuNode::controlModeCallback(const std_msgs::UInt8::ConstPtr& ctrl_mode_msg){
	
	/* Gather the Control Mode */
	controlMode.data = ctrl_mode_msg->data;
	
}

void MenuNode::stringPrintCallback(const std_msgs::String::ConstPtr& string_msg){

	/* Write message in the "selection" line */
	mvprintw(LINES - 1, 2, "                                                                         ");	
	mvprintw(LINES - 1, 2, string_msg->data.c_str());
	
	refresh();
	
}


/** Callbacks to capture data from the sensors **/
void MenuNode::gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& pos ){

	gpsPos_.status = pos->status;
	gpsPos_.latitude = pos->latitude;
	gpsPos_.longitude = pos->longitude;
	
	gpsTimer = ACTIVE_TIME;

}

void MenuNode::gpsVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel ){
	
	gpsVel_.linear = vel->linear;
	gpsVel_.angular = vel->angular;
	
}

void MenuNode::gpsHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head ){

	gpsHead_.theta = head->theta;

}
	
void MenuNode::gyroVelocityCallback(const geometry_msgs::Twist::ConstPtr& vel ){

	gyroVel_.angular = vel->angular;
	
	gyroTimer = ACTIVE_TIME;

}
	
void MenuNode::magnetometerHeadingCallback(const geometry_msgs::Pose2D::ConstPtr& head ){
	
	magHead_.theta = head->theta;
	
	magTimer = ACTIVE_TIME;
	
}
	
void MenuNode::roc4VelocityCallback(const geometry_msgs::Twist::ConstPtr& vel ){
	
	roc4Vel_.linear = vel->linear;
	roc4Vel_.angular = vel->angular;
	
	roburoc4Timer = ACTIVE_TIME;
	
}

/* Callback to update the local copy of the states */
void MenuNode::statesCallback(const roburoc4::States::ConstPtr& newStates){
	
	/* Update the local copy of the states */
	states_.x = newStates->x;
	states_.y = newStates->y;
	states_.theta = newStates->theta;
	states_.omega = newStates->omega;
	states_.v = newStates->v;
	
	stateEstTimer = ACTIVE_TIME;

}


