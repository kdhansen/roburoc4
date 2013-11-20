#include <telegram.hpp>

/*--------------------------------------------------------------------*/
/* General Telegram Class */
/*--------------------------------------------------------------------*/
Telegram::Telegram() { }

Telegram::Telegram( unsigned char _header ) {
	header_ = _header;
}

Telegram::~Telegram() { }

unsigned char Telegram::getHeader() {
	return header_;
}

/*--------------------------------------------------------------------*/
/* Action Telegram class */
/*--------------------------------------------------------------------*/
ActionTelegram::ActionTelegram() { }

ActionTelegram::ActionTelegram( unsigned char _header, unsigned char _action) {
	header_ = _header;
	action_ = _action;
}

ActionTelegram::~ActionTelegram() { }

unsigned char ActionTelegram::getAction() {
	return action_;
}
