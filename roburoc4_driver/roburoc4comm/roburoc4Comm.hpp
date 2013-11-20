#ifndef ROBUROC4COMM_HPP
#define ROBUROC4COMM_HPP

#include <boost/asio/ip/udp.hpp>
#include <boost/asio.hpp>

#include <pure_telegrams.hpp> 	// The communication protocol to the robot

#include <string>


// Define a specific number related to the specific telegram type rather than service
#define RR4COM_UNKNOWN_TELEGRAM 		0x00
#define RR4COM_PROPERTY_IOCARD  		0x10
#define RR4COM_PROPERTY_TELEMETER  		0x11
#define RR4COM_PROPERTY_BATTERY 		0x12
#define RR4COM_PROPERTY_LOCALIZATION 	0x13
#define RR4COM_PROPERTY_DIFFERENTIAL	0x14
#define RR4COM_STATUS_IOCARD			0x30
#define RR4COM_STATUS_TELEMETER			0x31
#define RR4COM_STATUS_BATTERY			0x32
#define RR4COM_STATUS_LOCALIZATION		0x33
#define RR4COM_STATUS_DIFFERENTIAL		0x34

using boost::asio::ip::udp;
using namespace std;

class Roburoc4Comm {
	
	public:
		Roburoc4Comm(std::string serverAddress, int serverPort);
		~Roburoc4Comm();
		
		int subscribe(short sensor, unsigned char period);
		int requestProperties(short sensor);

		int setVelocity(float linearVelocity, float angularVelocity);
		int setDigitalOutput(unsigned char output);
		int setPosition(double x, double y, double z);

		int getNewTelegramType();
		
		int getNumberIOCardPropertiesTelegrams();
		int getNumberTelemeterPropertiesTelegrams();
		int getNumberBatteryPropertiesTelegrams();
		int getNumberLocalizationPropertiesTelegrams();
		int getNumberDifferentialPropertiesTelegrams();
		
		int getNumberIOCardStatusTelegrams();
		int getNumberTelemeterStatusTelegrams();
		int getNumberBatteryStatusTelegrams();
		int getNumberLocalizationStatusTelegrams();
		int getNumberDifferentialStatusTelegrams();

		IOCardPropertiesTelegram getIOCardPropertiesTelegram();
		TelemeterPropertiesTelegram getTelemeterPropertiesTelegram();
		BatteryPropertiesTelegram getBatteryPropertiesTelegram();
		LocalizationPropertiesTelegram getLocalizationPropertiesTelegram();
		DifferentialPropertiesTelegram getDifferentialPropertiesTelegram();
		
		IOCardStatusTelegram getIOCardStatusTelegram();
		TelemeterStatusTelegram getTelemeterStatusTelegram();
		BatteryStatusTelegram getBatteryStatusTelegram();
		LocalizationStatusTelegram getLocalizationStatusTelegram();
		DifferentialStatusTelegram getDifferentialStatusTelegram();	
		
	private:
		boost::asio::io_service io_service;		// The IO service
		udp::endpoint receiver_endpoint;		// The Endpoint (the robot)	
		udp::socket *udp_socket;
		
	/* Vectors to contain the different messages returned (Only for those implemented) */
		list<IOCardPropertiesTelegram> iOCardPropertiesTelegram;
		list<TelemeterPropertiesTelegram> telemeterPropertiesTelegram;
		list<BatteryPropertiesTelegram> batteryPropertiesTelegram;
		list<LocalizationPropertiesTelegram> localizationPropertiesTelegram;
		list<DifferentialPropertiesTelegram> differentialPropertiesTelegram;
		
		list<IOCardStatusTelegram> iOCardStatusTelegram;
		list<TelemeterStatusTelegram> telemeterStatusTelegram;
		list<BatteryStatusTelegram> batteryStatusTelegram;
		list<LocalizationStatusTelegram> localizationStatusTelegram;
		list<DifferentialStatusTelegram> differentialStatusTelegram;	
		
};

#endif
