#include <termios.h> 	/* POSIX terminal control definitions */
#include <fcntl.h>   	/* File control definitions */
#include <string.h>		/* String functions for memset */
#include <stdio.h>		/* Printf thing.. */
#include <serialCommunicator.hpp>

/* Constructor */
serialCommunicator::serialCommunicator(string port, int baud) {
	
	speed_t baudRate;
	
	/* Open the serial port */
	serialPort = open(port.c_str(),O_RDWR | O_NDELAY); // Open the port
	
	if (serialPort == -1)
		throw string("Unable to open port: "+port);

	/* Find baud rate */
	if ( baud == 1800 ) {
		baudRate = B1800;
	} else if ( baud == 2400 ) {
		baudRate = B2400;
	} else if ( baud == 4800 ) {
		baudRate = B4800;
	} else if ( baud == 9600 ) {
		baudRate = B9600;
	} else if ( baud == 19200 ) {
		baudRate = B19200;
	} else if ( baud == 38400 ) {
		baudRate = B38400;
	} else if ( baud == 115200 ) {
		baudRate = B115200;
	} else {
		throw string("Baud rate unknown!");
	}
	
	/* Generate the settings */
	struct termios port_settings;      // structure to store the port settings in
	memset(&port_settings,0,sizeof(struct termios));
	
	cfsetispeed(&port_settings, baudRate);    // set baud rates
	cfsetospeed(&port_settings, baudRate);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	
	tcsetattr(serialPort, TCSANOW, &port_settings);    // apply the settings to the port
}

/* Destructor */
serialCommunicator::~serialCommunicator() {
	
	/* Close the serial port */
	close(serialPort);
	
}

/* Function to get a single line from the serial port */
string serialCommunicator::getLine() {

	int nBytes;
	char readBuffer[200];
	string returnString;
	
	/* Read new data from the serial port */
	nBytes = read(serialPort,readBuffer,200);
	
	/* Append the new data to the tempBuffer */
	if (nBytes > 0){
		tempBuffer.append(readBuffer,nBytes);	
	}
	
	/* If newline exists in the tempBuffer extract the line otwervise let an emtu string be returned */
	if ( tempBuffer.find(LINE_SHIFT_CHAR) <= tempBuffer.length() ){
		
		/* Extract the string until the first newline '\n' */
		returnString = tempBuffer.substr(0,tempBuffer.find(LINE_SHIFT_CHAR));
		
		/* Remove the extracted data from the tempBuffer */
		tempBuffer = tempBuffer.substr(tempBuffer.find(LINE_SHIFT_CHAR)+1);
	
	}
	
	/* Return the extracted substring */
	return returnString;
}

int serialCommunicator::getChar(unsigned char *c) {
	
	int nBytes;
	char readBuffer[200];
	
	/* Read new data from the serial port */
	nBytes = read(serialPort,readBuffer,200);
	
	/* Append the new data to the tempBuffer */
	if (nBytes > 0){
		tempBuffer.append(readBuffer,nBytes);	
	}
	
	/* Check if the buffer contains any data */
	if ( tempBuffer.size() > 0 ){
		 *c = tempBuffer.at(0);
		 tempBuffer.erase(0,1);
		return 1;
	}else{
		return 0;
	}
	

}


/* send a string of data to the serial port, optional enter a char delay in micro seconds */
int serialCommunicator::send(string data, int charDelay) {
	

	/* If no char delay is wanted send the string as it is */
	if ( charDelay == 0 ) {
		write(serialPort, data.c_str(), data.length());
	}
	
	/* If a char delay is wanted, write the chars to the serial port one at a time */
	else {
		for ( unsigned int j = 0 ; j < data.length() ; j++) {
			/* Extract the char from the string */
			char a = data.at(j);
			
			/* Write the char to the port */
			write(serialPort, &a, 1);
			
			/* Wait for chardelay in micro seconds */
			usleep(charDelay);	
		}
	}
	
	return 1;
}
