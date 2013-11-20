#ifndef COMMAND_TELEGRAM_HPP
#define COMMAND_TELEGRAM_HPP

#include "pure_protocol/telegram.hpp"

/*--------------------------------------------------------------------*/
/* General Command Telegram class */
/*--------------------------------------------------------------------*/
class CommandTelegram : public Telegram {
	
	public:
	/* Constructor & Destructor */
	CommandTelegram();
	~CommandTelegram();
	
	protected:
	/* Attributes */
	short serviceID_;
	
	private:
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<Telegram>(*this);
        ar & serviceID_;
    }
};

BOOST_CLASS_IMPLEMENTATION(CommandTelegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* IOCard Command Telegram class */
//~ No Analog outputs: 0
//~ No Digital outputs: 8
/*--------------------------------------------------------------------*/
class IOCardCommandTelegram : public CommandTelegram {
	
	public:
	/* Constructor & Destructor */
	IOCardCommandTelegram();
	IOCardCommandTelegram(unsigned char _DO);
	~IOCardCommandTelegram();
	
	private:
	/* Attributes */
	unsigned char DO_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<CommandTelegram>(*this);
        ar & DO_;
	}
};

BOOST_CLASS_IMPLEMENTATION(IOCardCommandTelegram, boost::serialization::object_serializable);

/*--------------------------------------------------------------------*/
/* Differential Command Telegram class */
/*--------------------------------------------------------------------*/
class DifferentialCommandTelegram : public CommandTelegram {
	
	public:
	/* Constructor & Destructor */
	DifferentialCommandTelegram();
	DifferentialCommandTelegram( float _targetLinearSpeed, float _targetAngularSpeed);
	~DifferentialCommandTelegram();
	
	private:
	/* Attributes */
	float targetLinearSpeed_;
	float targetAngularSpeed_; 	 	
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<CommandTelegram>(*this);
        ar & targetLinearSpeed_;
        ar & targetAngularSpeed_;
    }
};

BOOST_CLASS_IMPLEMENTATION(DifferentialCommandTelegram, boost::serialization::object_serializable);

#endif
