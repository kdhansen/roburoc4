#include "pure_protocol/status_telegram.hpp"


/*--------------------------------------------------------------------*/
/* Subscribe Telegram */
/*--------------------------------------------------------------------*/
SubscribeTelegram::SubscribeTelegram(){
}

SubscribeTelegram::SubscribeTelegram( unsigned char _header, short _serviceID, unsigned char _period) {

	header_ = _header;
	action_ = PURE_ACTION_INSERT;
	notificationServiceInstanceNumber_ = 0x01;
	serviceID_ = _serviceID;
	period_ = _period;	
}

SubscribeTelegram::~SubscribeTelegram(){
	
}
	

/*--------------------------------------------------------------------*/
/* General Status Telegram */
/*--------------------------------------------------------------------*/

StatusTelegram::StatusTelegram(){
	
}

StatusTelegram::~StatusTelegram(){
	
}

short StatusTelegram::getServiceID(){

	return serviceID_;
	
}

unsigned long int StatusTelegram::getTimeStamp(){
	
	return timeStamp_;
	
}


/*--------------------------------------------------------------------*/
/* IOCard status */
/*--------------------------------------------------------------------*/
IOCardStatusTelegram::IOCardStatusTelegram(){
	
}

IOCardStatusTelegram::~IOCardStatusTelegram(){
	
}

float IOCardStatusTelegram::getAnalogInput(int noAI){
	
	if (0 <= noAI && noAI <= 8)
		return AI_[noAI];
	else
		return 0;
	
}

unsigned char IOCardStatusTelegram::getDigitalInput(int noDI){
	
	if (0 <= noDI && noDI <= 2)
		return DI_[noDI];
	else
		return 0;
		
}


/*--------------------------------------------------------------------*/
/* Telemeter status */
/*--------------------------------------------------------------------*/
TelemeterStatusTelegram::TelemeterStatusTelegram(){
	
}

TelemeterStatusTelegram::~TelemeterStatusTelegram(){
	
}

float TelemeterStatusTelegram::getDist(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return dist_[teleServiceNo];
	else
		return 0;
}



/*--------------------------------------------------------------------*/
/* Battery Status Telegram class */
/*--------------------------------------------------------------------*/
BatteryStatusTelegram::BatteryStatusTelegram(){
	
}

BatteryStatusTelegram::~BatteryStatusTelegram(){
	
}

unsigned char BatteryStatusTelegram::getState(){

	return state_;
	
}

unsigned char BatteryStatusTelegram::getRemaining(){
	
	return remaining_;
	
}


/*--------------------------------------------------------------------*/
/* Localization status */
/*--------------------------------------------------------------------*/

LocalizationStatusTelegram::LocalizationStatusTelegram(){
	
}

LocalizationStatusTelegram::~LocalizationStatusTelegram(){
	
}

double LocalizationStatusTelegram::getX(){
	
	return x_;
	
}

double LocalizationStatusTelegram::getY(){
	
	return y_;
	
}

double LocalizationStatusTelegram::getTheta(){
	
	return theta_;
	
}

unsigned int LocalizationStatusTelegram::getState(){
	
	return state_;
	
}

/*--------------------------------------------------------------------*/
/* Differential status */
/*--------------------------------------------------------------------*/
DifferentialStatusTelegram::DifferentialStatusTelegram(){
	
}

DifferentialStatusTelegram::~DifferentialStatusTelegram(){
	
}

float DifferentialStatusTelegram::getTargetLinearSpeed(){
	
	return targetLinearSpeed_;
	
}

float DifferentialStatusTelegram::getTargetAngularSpeed(){
	
	return targetAngularSpeed_;
	
}

float DifferentialStatusTelegram::getCurrentLinearSpeed(){
	
	return currentLinearSpeed_;
	
}

float DifferentialStatusTelegram::getCurrentAngularSpeed(){
	
	return currentAngularSpeed_;
	
}
