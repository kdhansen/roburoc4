#ifndef SERIALCOMMUNICATOR_HPP
#define SERIALCOMMUNICATOR_HPP

#define LINE_SHIFT_CHAR 		13	// CR is used

#include <iostream>
#include <string>
#include <stdexcept>

using namespace std;

class serialCommunicator {
	
	private:
	int serialPort;
	string tempBuffer;
	
	public:
	serialCommunicator(string port, int baud);
	~serialCommunicator();
	string getLine();
	int getChar(unsigned char* c);
	int send(string data, int charDelay=0);
};

#endif
