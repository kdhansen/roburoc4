

#include "pure_protocol/replace_telegram.hpp"

/*--------------------------------------------------------------------*/
/* Localization Replace Telegram class */
/*--------------------------------------------------------------------*/
LocalizationReplaceTelegram::LocalizationReplaceTelegram(){
	
}

LocalizationReplaceTelegram::LocalizationReplaceTelegram( unsigned char _header, double _x, double _y, double theta){
	header_ = _header;
	action_ = PURE_ACTION_REPLACE;
	target_ = PURE_SERVICE_ID_LOCALIZATION;
	timeStamp_ = 1337;	// Not used but should be something different from 0!
	x_ = _x;
	y_ = _y;
	theta_ = theta;
}

LocalizationReplaceTelegram::~LocalizationReplaceTelegram(){
	
}

