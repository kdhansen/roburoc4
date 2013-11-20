#include <properties_telegram.hpp>

/*--------------------------------------------------------------------*/
/* General Request Properties Telegram Class */
/*--------------------------------------------------------------------*/
RequestPropertiesTelegram::RequestPropertiesTelegram(){

}

RequestPropertiesTelegram::RequestPropertiesTelegram( unsigned char _header, short _serviceID){
	header_ = _header;
	action_ = PURE_ACTION_GET;
	serviceID_ = _serviceID;
}

RequestPropertiesTelegram::~RequestPropertiesTelegram(){
}

short RequestPropertiesTelegram::getServiceID(){
	return serviceID_;
}

/*--------------------------------------------------------------------*/
/* General Response Properties Telegram Class */
/*--------------------------------------------------------------------*/
ResponsePropertiesTelegram::ResponsePropertiesTelegram(){
}

ResponsePropertiesTelegram::~ResponsePropertiesTelegram(){
}

short ResponsePropertiesTelegram::getTarget(){
		return target_;
}

unsigned char ResponsePropertiesTelegram::getError(){
	return error_;
}

/*--------------------------------------------------------------------*/
/* Directory Properties Telegram Class */
/*--------------------------------------------------------------------*/
// TODO: Implement this

/*--------------------------------------------------------------------*/
/* Notification Properties Telegram Class */
/*--------------------------------------------------------------------*/
// TODO: Implement this

/*--------------------------------------------------------------------*/
/* IOCard Properties Telegram Class */
/*--------------------------------------------------------------------*/
IOCardPropertiesTelegram::IOCardPropertiesTelegram(){
}

IOCardPropertiesTelegram::~IOCardPropertiesTelegram(){
}

int IOCardPropertiesTelegram::getNumberAnalogInputs(){
	return numberAnalogInputs_;
}

int IOCardPropertiesTelegram::getNumberAnalogOutpts(){
	return numberAnalogOutputs_;
}

int IOCardPropertiesTelegram::getNumberDigitalInputs(){
	return numberDigitalInputs_;
}

int IOCardPropertiesTelegram::getNumberDigitalOutputs(){
	return numberDigitalOutputs_;
}

/*--------------------------------------------------------------------*/
/* Telemeter Properties Telegram Class */
/*--------------------------------------------------------------------*/
TelemeterPropertiesTelegram::TelemeterPropertiesTelegram(){
}

TelemeterPropertiesTelegram::~TelemeterPropertiesTelegram(){
}

int TelemeterPropertiesTelegram::getNumberTelemeterServices(){
	return numberOfTelemeterServices_;
}

float TelemeterPropertiesTelegram::getX(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return X_[teleServiceNo];
	else
		return 0;
}

float TelemeterPropertiesTelegram::getY(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return Y_[teleServiceNo];
	else
		return 0;
}

float TelemeterPropertiesTelegram::getT(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return T_[teleServiceNo];
	else
		return 0;	
}

float TelemeterPropertiesTelegram::getFoV(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return FoV_[teleServiceNo];
	else
		return 0;	
}

float TelemeterPropertiesTelegram::getminD(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return minD_[teleServiceNo];
	else
		return 0;	
}

float TelemeterPropertiesTelegram::getmaxD(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return maxD_[teleServiceNo];
	else
		return 0;	
}

float TelemeterPropertiesTelegram::getdist(int teleServiceNo){
	if (teleServiceNo == 0 || teleServiceNo == 1)
		return dist_[teleServiceNo];
	else
		return 0;	
}

/*--------------------------------------------------------------------*/
/* Battery Properties Telegram Class */
/*--------------------------------------------------------------------*/
BatteryPropertiesTelegram::BatteryPropertiesTelegram(){
}

BatteryPropertiesTelegram::~BatteryPropertiesTelegram(){
}

float BatteryPropertiesTelegram::getNominalVoltage(){
	return nominalVoltage_;
}

float BatteryPropertiesTelegram::getEnergy(){
	return energy_;
}

unsigned char BatteryPropertiesTelegram::getMinimumPowerPercentage(){
	return minimumPowerPercentage_;
}

/*--------------------------------------------------------------------*/
/* Localization Properties Telegram Class */
/*--------------------------------------------------------------------*/
LocalizationPropertiesTelegram::LocalizationPropertiesTelegram(){
	
}

LocalizationPropertiesTelegram::~LocalizationPropertiesTelegram(){
	
}

double LocalizationPropertiesTelegram::getInitialX(){
	return x_;
}

double LocalizationPropertiesTelegram::getInitialY(){
	return y_;
}

double LocalizationPropertiesTelegram::getInitialTheta(){
	return theta_;
}

unsigned int LocalizationPropertiesTelegram::getState(){
	return state_;
}

/*--------------------------------------------------------------------*/
/* Differential Properties Telegram Class */
/*--------------------------------------------------------------------*/
DifferentialPropertiesTelegram::DifferentialPropertiesTelegram(){
	
}

DifferentialPropertiesTelegram::~DifferentialPropertiesTelegram(){
	
}

float DifferentialPropertiesTelegram::getTargetLinearSpeed(){
	return targetLinearSpeed_;
}

float DifferentialPropertiesTelegram::getTargetAngularSpeed(){
	return targetAngularSpeed_;
}

float DifferentialPropertiesTelegram::getCurrentLinearSpeed(){
	return currentLinearSpeed_;
}

float DifferentialPropertiesTelegram::getCurrentAngularSpeed(){
	return currentAngularSpeed_;
}

float DifferentialPropertiesTelegram::getMaximumLinearSpeed(){
	return maximumLinearSpeed_;
}

float DifferentialPropertiesTelegram::getMinimumLinearSpeed(){
	return minimumLinearSpeed_;
}

float DifferentialPropertiesTelegram::getMaximumAngularSpeed(){
	return maximumAngularSpeed_;
}

float DifferentialPropertiesTelegram::getMinimumAngularSpeed(){
	return minimumAngularSpeed_;
}

float DifferentialPropertiesTelegram::getMaximumLinearAcceleration(){
	return maximumLinearAcceleration_;
}

float DifferentialPropertiesTelegram::getMinimumLinearAcceleration(){
	return minimumLinearAcceleration_;
}

float DifferentialPropertiesTelegram::getMaximumAngularAcceleration(){
	return maximumAngularAcceleration_;
}

float DifferentialPropertiesTelegram::getMinimumAngularAcceleration(){
	return minimumAngularAcceleration_;
}

float DifferentialPropertiesTelegram::getWidth(){
	return width_;
}
