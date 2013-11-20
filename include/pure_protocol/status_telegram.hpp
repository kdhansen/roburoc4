#ifndef STATUS_TELEGRAM_HPP
#define STATUS_TELEGRAM_HPP

#include "pure_protocol/telegram.hpp"

/*--------------------------------------------------------------------*/
/* Subscribe Telegram class */
/*--------------------------------------------------------------------*/
class SubscribeTelegram : public ActionTelegram {
	
	public:
	/* Constructor & Destructor */
	SubscribeTelegram();
	SubscribeTelegram( unsigned char _header, short _serviceID, unsigned char _period);
	~SubscribeTelegram();
	
	private:
	/* Attributes */
	short notificationServiceInstanceNumber_;
	short serviceID_;
	unsigned char period_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<ActionTelegram>(*this);
        ar & notificationServiceInstanceNumber_;
        ar & serviceID_;
        ar & period_;
    }
};

BOOST_CLASS_IMPLEMENTATION(SubscribeTelegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* General Status Telegram class */
/*--------------------------------------------------------------------*/
class StatusTelegram : public Telegram {
	
	public:
	/* Constructor & Destructor */
	StatusTelegram();
	~StatusTelegram();
	
	/* Memberfunctions */
	short getServiceID();
	unsigned long int getTimeStamp();
	
	private:
	/* Attributes */
	short serviceID_;
	unsigned long int timeStamp_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<Telegram>(*this);
        ar & serviceID_;
        ar & timeStamp_;
    }
};

BOOST_CLASS_IMPLEMENTATION(StatusTelegram, boost::serialization::object_serializable);

/*--------------------------------------------------------------------*/
/* IOCard status */
//~ No Analog inputs: 9
//~ No Digital inputs: 22
/*--------------------------------------------------------------------*/
class IOCardStatusTelegram : public StatusTelegram {
	
	public:
	/* Constructor & Destructor */
	IOCardStatusTelegram();
	~IOCardStatusTelegram();
	
	/* Memberfunctions */
	float getAnalogInput(int noAI);
	unsigned char getDigitalInput(int noDI);
	
	private:
	/* Attributes */
	float AI_[9];
	unsigned char DI_[3];
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<StatusTelegram>(*this);
        ar & AI_[0];
        ar & AI_[1];
        ar & AI_[2];
        ar & AI_[3];
        ar & AI_[4];
        ar & AI_[5];
        ar & AI_[6];
        ar & AI_[7];
        ar & AI_[8];
        ar & DI_[0];
        ar & DI_[1];
        ar & DI_[2];
    }
};

BOOST_CLASS_IMPLEMENTATION(IOCardStatusTelegram, boost::serialization::object_serializable);

/*--------------------------------------------------------------------*/
/* Telemeter status */
/*--------------------------------------------------------------------*/
class TelemeterStatusTelegram : public StatusTelegram {
	
	public:
	/* Constructor & Destructor */
	TelemeterStatusTelegram();
	~TelemeterStatusTelegram();
	
	/* Memberfunctions */
	float getDist(int teleServiceNo);
	
	private:
	/* Attributes */
	float dist_[2];
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<StatusTelegram>(*this);
        ar & dist_[0];
        ar & dist_[1];
    }
};

BOOST_CLASS_IMPLEMENTATION(TelemeterStatusTelegram, boost::serialization::object_serializable);


/*--------------------------------------------------------------------*/
/* Battery Status Telegram class */
/*--------------------------------------------------------------------*/
class BatteryStatusTelegram : public StatusTelegram {
	
	public:
	/* Constructor & Destructor */
	BatteryStatusTelegram();
	~BatteryStatusTelegram();
	
	/* Memberfunctions */
	unsigned char getState();
	unsigned char getRemaining();
	
	private:
	/* Attributes */
	unsigned char state_;
	unsigned char remaining_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<StatusTelegram>(*this);
        ar & state_;
        ar & remaining_;
    }
};

BOOST_CLASS_IMPLEMENTATION(BatteryStatusTelegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* Localization status */
/*--------------------------------------------------------------------*/
class LocalizationStatusTelegram : public StatusTelegram {
	
	public:
	/* Constructor & Destructor */
	LocalizationStatusTelegram();
	~LocalizationStatusTelegram();
	
	/* Memberfunctions */
	double getX();
	double getY();
	double getTheta();
	unsigned int getState();
		
	private:
	/* Attributes */
	double x_;
	double y_;
	double theta_;
	unsigned int state_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<StatusTelegram>(*this);
        ar & x_;
        ar & y_;
		ar & theta_;
        ar & state_;
    }
};

BOOST_CLASS_IMPLEMENTATION(LocalizationStatusTelegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* Differential status */
/*--------------------------------------------------------------------*/
class DifferentialStatusTelegram : public StatusTelegram {
	
	public:
	/* Constructor & Destructor */
	DifferentialStatusTelegram();
	~DifferentialStatusTelegram();
	
	/* Memberfunctions */
	float getTargetLinearSpeed();
	float getCurrentLinearSpeed();
	float getTargetAngularSpeed();
	float getCurrentAngularSpeed();
		
	private:
	/* Attributes */
	float targetLinearSpeed_;
	float currentLinearSpeed_;
	float targetAngularSpeed_;
	float currentAngularSpeed_;

	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<StatusTelegram>(*this);
        ar & targetLinearSpeed_;
        ar & currentLinearSpeed_;
		ar & targetAngularSpeed_;
        ar & currentAngularSpeed_;
    }
};

BOOST_CLASS_IMPLEMENTATION(DifferentialStatusTelegram, boost::serialization::object_serializable);

#endif
