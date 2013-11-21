// Communications features for the driver for the RobuROC4.
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

#include <roburoc4Comm.hpp>

#include <iostream>
#include <string>

/* Constructor */
Roburoc4Comm::Roburoc4Comm(std::string serverAddress, int serverPort) {

	/* Generate port name as string */
	std::stringstream ss;
	ss << serverPort;

	/* Look up the service */
	udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), serverAddress.c_str(), ss.str().c_str());
    receiver_endpoint = *resolver.resolve(query);
    
    /* Open an outgoing socket */
    udp_socket = new udp::socket(io_service);
    udp_socket->open(udp::v4());
    
}

/* Destructor */
Roburoc4Comm::~Roburoc4Comm(){
	
	/* Delete the socket */
	delete udp_socket;
	
}

/* Function to subscribe to a sensor */
int Roburoc4Comm::subscribe(short sensor, unsigned char period){
	
	/* Generate the packet to send */
	SubscribeTelegram subscribeTelegram(0x02,sensor,period);
	
	/* Stringstream and binary archive for transmitting the packet */
	std::stringstream ss_out;
	boost::archive::binary_oarchive oa(ss_out,1);
	
	/* Serialize the packet */
	oa << subscribeTelegram;

	/* Send the message */
	udp_socket->send_to(boost::asio::buffer(ss_out.str().c_str(),ss_out.str().size()), receiver_endpoint);

	//~ fprintf(stderr, "Sent data: ");
	//~ for (unsigned int i = 0; i < ss_out.str().size(); i++) {
	  //~ fprintf(stderr, "%02x ",(unsigned char)ss_out.str().at(i));
	//~ }
	//~ fprintf(stderr, "\n");


	return 1;
}

/* Request properties from a sensor */
int Roburoc4Comm::requestProperties(short sensor){
	
	/* Generate the packet to send */
	RequestPropertiesTelegram requestPropertiesTelegram(0x02,sensor);
	
	/* Stringstream and binary archive for transmitting the packet */
	std::stringstream ss_out;
	boost::archive::binary_oarchive oa(ss_out,1);
	
	/* Serialize the packet */
	oa << requestPropertiesTelegram;

	/* Send the message */
	udp_socket->send_to(boost::asio::buffer(ss_out.str().c_str(),ss_out.str().size()), receiver_endpoint);

	return 1;
}


/* Function to set velocity */
int Roburoc4Comm::setVelocity(float linearVelocity, float angularVelocity){
	
	/* Generate set velocity packet */
	DifferentialCommandTelegram differentialCommandTelegram(linearVelocity,angularVelocity);
	
	/* Stringstream and binary archive for transmitting the packet */
	std::stringstream ss_out;
	boost::archive::binary_oarchive oa(ss_out,1);
	
	/* Serialize the packet */
	oa << differentialCommandTelegram;

	/* Send the message */
	udp_socket->send_to(boost::asio::buffer(ss_out.str().c_str(),ss_out.str().size()), receiver_endpoint);
	
	return 1;
}

/* Function to set digital output */
int Roburoc4Comm::setDigitalOutput(unsigned char output){
	
	/* Generate set Digital output command */
	IOCardCommandTelegram ioCardCommandTelegram(output);
	
	/* Stringstream and binary archive for transmitting the packet */
	std::stringstream ss_out;
	boost::archive::binary_oarchive oa(ss_out,1);
	
	/* Serialize the packet */
	oa << ioCardCommandTelegram;

	/* Send the message */
	udp_socket->send_to(boost::asio::buffer(ss_out.str().c_str(),ss_out.str().size()), receiver_endpoint);
	
	return 1;
}

/* Function to set the position in the RobuROC4 */
int Roburoc4Comm::setPosition(double x, double y, double z){
	
	/* Generate the locilation replace telegram */
	LocalizationReplaceTelegram locReplaceTelegram(0x01,x,y,z);
	
	/* Stringstream and binary archive for transmitting the packet */
	std::stringstream ss_out;
	boost::archive::binary_oarchive oa(ss_out,1);
	
	/* Serialize the packet */
	oa << locReplaceTelegram;

	/* Send the message */
	udp_socket->send_to(boost::asio::buffer(ss_out.str().c_str(),ss_out.str().size()), receiver_endpoint);
	
	return 1;
	
}


/* Function to read the data from the socket, parse the data, and store them internally to be red from outside */
int Roburoc4Comm::getNewTelegramType(){
	
	int return_value = RR4COM_UNKNOWN_TELEGRAM;
	
	/* Buffers*/
	boost::array<char, 256> recv_buf;
	std::stringstream ss_in;
	std::stringstream ss_tmp;
	
	/* The input archive for deserilization */	
	boost::archive::binary_iarchive ia(ss_in,1);
	
	/* Wait for data on the socket */
    size_t len = udp_socket->receive_from(boost::asio::buffer(recv_buf), receiver_endpoint);

	/* Put the data into the stringstream */
	ss_in.write(recv_buf.data(),len);
	
	/* And into a string */
	std::string inputMessage = ss_in.str();

	/*
	fprintf(stderr, "Received data: ");
	for (unsigned int i = 0; i < ss_in.str().size(); i++) {
	  fprintf(stderr, "%02x ",(unsigned char)ss_in.str().at(i));
	}
	fprintf(stderr, "\n");
	*/

	/* Deserialize to Telegram telegram first to get type */
	Telegram telegram;
	ss_in.str(""); ss_in << inputMessage.substr(0,1); // The first byte only : to seperate the messages by header
	ia >> telegram;
	
	/* HEADER=0x00 => ERROR */
	if (telegram.getHeader() == PURE_HEADER_ERROR) {
			std::cout << "Roburoc4Node: Error message received" << std::endl;
	}
	
	/* HEADER=0xFF => Status update */
	else if (telegram.getHeader() == PURE_HEADER_STATUS) {
		
		StatusTelegram statusTelegram;
		ss_in.str(""); ss_in << inputMessage.substr(0,11);	// Use the first 11 bytes (header + service_id + timestamp) to be able to generate a statusTelegram and extract service_id
		ia >> statusTelegram;
		
		ss_in.str(""); ss_in << inputMessage;				// In all cases the different telegrams is deserialized from the total message.		

		/* IOCard status */
		if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_IOCARD ) {
			IOCardStatusTelegram tmp_io_stat;
			ia >> tmp_io_stat;
			iOCardStatusTelegram.push_back(tmp_io_stat);
			
			return_value = RR4COM_STATUS_IOCARD;	
		}
		
		/* TELEMETER status */
		else if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_TELEMETER ) {
			TelemeterStatusTelegram tmp_telemeter_stat;
			ia >> tmp_telemeter_stat;
			telemeterStatusTelegram.push_back(tmp_telemeter_stat);
			
			return_value = RR4COM_STATUS_TELEMETER;
		}
	
		/* BATTERY status */
		else if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_BATTERY ) {
			BatteryStatusTelegram tmp_bat_stat;
			ia >> tmp_bat_stat;
			batteryStatusTelegram.push_back(tmp_bat_stat);
			
			return_value = RR4COM_STATUS_BATTERY;		
		}
		
		/* LOCALIZATION status */
		else if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_LOCALIZATION ) {
			LocalizationStatusTelegram tmp_local_stat;
			ia >> tmp_local_stat;
			localizationStatusTelegram.push_back(tmp_local_stat);
			
			return_value = RR4COM_STATUS_LOCALIZATION;		
		}		
		
		/* DIFFERENTIAL status */
		else if ( statusTelegram.getServiceID() == PURE_SERVICE_ID_DIFFERENTIAL ) {
			
			DifferentialStatusTelegram tmp_diff_stat;
			ia >> tmp_diff_stat;
			differentialStatusTelegram.push_back(tmp_diff_stat);
			
			return_value = RR4COM_STATUS_DIFFERENTIAL;			
		}
		
		else {
			return_value = RR4COM_UNKNOWN_TELEGRAM;
		}
		
				
	}
	
	/* HEADER different from the special cases, it is a response to a command! */
	else {
		ActionTelegram actionTelegram;
		ss_in.str(""); ss_in << inputMessage.substr(0,2);		// Use the first 2 bytes (header + action) to be able to distinguish different the different actions
		ia >> actionTelegram;
		
		/* if GET */
		if (actionTelegram.getAction() == PURE_ACTION_GET) {
			ResponsePropertiesTelegram responsePropTelegram;
			ss_in.str(""); ss_in << inputMessage.substr(0,5);	// Use the first 5 bytes (header + action + target + error) to be able to see where the response is comming from
			ia >> responsePropTelegram;
			
			ss_in.str(""); ss_in << inputMessage;				// In all cases (from now on) the different telegrams is deserialized from the total message.
			
			
			/* IOCard properties */
			if (responsePropTelegram.getTarget() == PURE_SERVICE_ID_IOCARD ) {
				IOCardPropertiesTelegram tmp_io_prop;
				ia >> tmp_io_prop;
				iOCardPropertiesTelegram.push_back(tmp_io_prop);
				
				return_value = RR4COM_PROPERTY_IOCARD;
			}
			
			/* TELEMETER properties */
			else if (responsePropTelegram.getTarget() == PURE_SERVICE_ID_TELEMETER ) {
				TelemeterPropertiesTelegram tmp_telemeter_prop;
				ia >> tmp_telemeter_prop;
				telemeterPropertiesTelegram.push_back(tmp_telemeter_prop);
				
				return_value = RR4COM_PROPERTY_TELEMETER;
			}
			
			/* BATTERY properties */
			else if (responsePropTelegram.getTarget() == PURE_SERVICE_ID_BATTERY ) {
				BatteryPropertiesTelegram tmp_bat_prop;
				ia >> tmp_bat_prop;
				batteryPropertiesTelegram.push_back(tmp_bat_prop);
				
				return_value = RR4COM_PROPERTY_BATTERY;
			}
			
			/* LOCALIZATION properties */
			else if (responsePropTelegram.getTarget() == PURE_SERVICE_ID_LOCALIZATION ) {
				LocalizationPropertiesTelegram tmp_local_prop;
				ia >> tmp_local_prop;
				localizationPropertiesTelegram.push_back(tmp_local_prop);
				
				return_value = RR4COM_PROPERTY_LOCALIZATION;
			}
			
			/* DIFFERENTIAL properties */
			else if (responsePropTelegram.getTarget() == PURE_SERVICE_ID_DIFFERENTIAL ) {
				DifferentialPropertiesTelegram tmp_diff_prop;
				ia >> tmp_diff_prop;
				differentialPropertiesTelegram.push_back(tmp_diff_prop);
				
				return_value = RR4COM_PROPERTY_DIFFERENTIAL;	
			}
			
			/* Unkonwn service id! */
			else {
				return_value = RR4COM_UNKNOWN_TELEGRAM;
			}
						
		}
		
	}
			
	return return_value;

}			



/* Getter functions to get values from the lists cntaining the telegrams */

int Roburoc4Comm::getNumberIOCardPropertiesTelegrams(){
	return (int)iOCardPropertiesTelegram.size();
}

int Roburoc4Comm::getNumberTelemeterPropertiesTelegrams(){
	return (int)telemeterPropertiesTelegram.size();
}

int Roburoc4Comm::getNumberBatteryPropertiesTelegrams(){
	return (int)batteryPropertiesTelegram.size();
}

int Roburoc4Comm::getNumberLocalizationPropertiesTelegrams(){
	return (int)localizationPropertiesTelegram.size();
}

int Roburoc4Comm::getNumberDifferentialPropertiesTelegrams(){
	return (int)differentialPropertiesTelegram.size();
}


int Roburoc4Comm::getNumberIOCardStatusTelegrams(){
	return (int)iOCardStatusTelegram.size();
}

int Roburoc4Comm::getNumberTelemeterStatusTelegrams(){
	return (int)telemeterStatusTelegram.size();
}

int Roburoc4Comm::getNumberBatteryStatusTelegrams(){
	return (int)batteryStatusTelegram.size();
}

int Roburoc4Comm::getNumberLocalizationStatusTelegrams(){
	return (int)localizationStatusTelegram.size();
}

int Roburoc4Comm::getNumberDifferentialStatusTelegrams(){
	return (int)differentialStatusTelegram.size();
}



/* Getter functions ToDo: make some tests if zero elements exists in the lists! */
IOCardPropertiesTelegram Roburoc4Comm::getIOCardPropertiesTelegram(){
	IOCardPropertiesTelegram tmp = iOCardPropertiesTelegram.back();
	iOCardPropertiesTelegram.pop_back();
	return tmp;
}

TelemeterPropertiesTelegram Roburoc4Comm::getTelemeterPropertiesTelegram(){
	TelemeterPropertiesTelegram tmp = telemeterPropertiesTelegram.back();
	telemeterPropertiesTelegram.pop_back();
	return tmp;
}


BatteryPropertiesTelegram Roburoc4Comm::getBatteryPropertiesTelegram(){
	BatteryPropertiesTelegram tmp = batteryPropertiesTelegram.back();
	batteryPropertiesTelegram.pop_back();
	return tmp;
}

LocalizationPropertiesTelegram Roburoc4Comm::getLocalizationPropertiesTelegram(){
	LocalizationPropertiesTelegram tmp = localizationPropertiesTelegram.back();
	localizationPropertiesTelegram.pop_back();
	return tmp;
}

DifferentialPropertiesTelegram Roburoc4Comm::getDifferentialPropertiesTelegram(){
	DifferentialPropertiesTelegram tmp = differentialPropertiesTelegram.back();
	differentialPropertiesTelegram.pop_back();
	return tmp;
}


IOCardStatusTelegram Roburoc4Comm::getIOCardStatusTelegram(){
	IOCardStatusTelegram tmp = iOCardStatusTelegram.back();
	iOCardStatusTelegram.pop_back();
	return tmp;
}

TelemeterStatusTelegram Roburoc4Comm::getTelemeterStatusTelegram(){
	TelemeterStatusTelegram tmp = telemeterStatusTelegram.back();
	telemeterStatusTelegram.pop_back();
	return tmp;
}

BatteryStatusTelegram Roburoc4Comm::getBatteryStatusTelegram(){
	BatteryStatusTelegram tmp = batteryStatusTelegram.back();
	batteryStatusTelegram.pop_back();
	return tmp;
}

LocalizationStatusTelegram Roburoc4Comm::getLocalizationStatusTelegram(){
	LocalizationStatusTelegram tmp = localizationStatusTelegram.back();
	localizationStatusTelegram.pop_back();
	return tmp;
}

DifferentialStatusTelegram Roburoc4Comm::getDifferentialStatusTelegram(){
	DifferentialStatusTelegram tmp = differentialStatusTelegram.back();
	differentialStatusTelegram.pop_back();
	return tmp;
}
	











//~ /* Function get message DEPRICATED: Use getNewTelegramType instead */
//~ std::string Roburoc4Comm::getMessage(){
	//~ 
	//~ /* Buffers*/
	//~ boost::array<char, 256> recv_buf;
	//~ std::stringstream ss_in;
	//~ 
	//~ /* Wait for data on the socket */
    //~ size_t len = udp_socket->receive_from(boost::asio::buffer(recv_buf), receiver_endpoint);
//~ 
	//~ /* Put the data into the stringstream */
	//~ ss_in.write(recv_buf.data(),len);
//~ 
	//~ return ss_in.str();
//~ 
	/* TEMP CODE */

	//~ std::cout << "message received on socket... ";
    //~ std::cout.write(recv_buf.data(), len);
    //~ std::cout << std::endl;
    //~ 
    //~ for ( unsigned int i = 0 ; i < len ; i++ ){
		//~ printf("%02x ",recv_buf[i]);
	//~ } 
	//~ 
	//~ std::cout << std::endl;
	
	/* UNTIL HERE */
//~ }

