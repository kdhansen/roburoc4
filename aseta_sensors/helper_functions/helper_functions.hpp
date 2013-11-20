#ifndef HELPERFUNCTIONS_HPP
#define HELPERFUNCTIONS_HPP

#include <vector>
#include <string>
#include <stdio.h>
#include <ctime>

using namespace std;

/* Split a string by a given char */
void split(const string& s, char c, vector<string>& v);

/* Calculate the NMEA checksum of a NMEA string */
string calculateNMEAChecksum( std::string message );

/* Count number of a given char in a string */
int nChars( std::string message, unsigned char c );

/* Generate the current date in format: YYYYMMDDHHII */
string generateDate();


#endif
