#include "helper_functions/helper_functions.hpp"

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

/* Calculate the NMEA checksum */
string calculateNMEAChecksum( std::string message ) {
	
	// Loop through all chars to get a checksum
	int checksum = 0;
	char character;
	
	for( unsigned int i = 0; i < message.length() ; i++ ) {
		
		character = message.at(i);						// Extract the character at the i'th place
		
		if ( character == '$' ){						// Ignore the dollar sign
		
		} 
		else if ( character == '*' ){					// Stop processing before the asterisk
			break;
		} 
		else {											// XOR the checksum with this character's value
			if (checksum == 0){
				checksum = character;
			}
			else {
			checksum ^= character;		
			}
		}
	}
	
	// Return the checksum formatted as a two-character hexadecimal
	char output[3];
	sprintf(output,"%02X",checksum);
		
	return string(output);
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
