#include "pure_protocol/command_telegram.hpp"
/*--------------------------------------------------------------------*/
/* CommandTelegram */
/*--------------------------------------------------------------------*/
CommandTelegram::CommandTelegram(){	
}

CommandTelegram::~CommandTelegram(){
	
}

/*--------------------------------------------------------------------*/
/* CommandTelegram */
/*--------------------------------------------------------------------*/
IOCardCommandTelegram::IOCardCommandTelegram(){
	
}

IOCardCommandTelegram::IOCardCommandTelegram(unsigned char _DO){
	header_ = 0xff;
	serviceID_ = PURE_SERVICE_ID_IOCARD;
	DO_ = _DO;
}

IOCardCommandTelegram::~IOCardCommandTelegram(){
	
}



/*--------------------------------------------------------------------*/
/* DifferentialCommandTelegram */
/*--------------------------------------------------------------------*/
DifferentialCommandTelegram::DifferentialCommandTelegram() {
	
}

DifferentialCommandTelegram::DifferentialCommandTelegram( float _targetLinearSpeed, float _targetAngularSpeed) {

	header_ = 0xff;
	serviceID_ = PURE_SERVICE_ID_DIFFERENTIAL;
	targetLinearSpeed_ = _targetLinearSpeed;
	targetAngularSpeed_ = _targetAngularSpeed;	

}

DifferentialCommandTelegram::~DifferentialCommandTelegram(){
	
}
