#ifndef PROPERTIES_TELEGRAM_HPP
#define PROPERTIES_TELEGRAM_HPP

#include "pure_protocol/telegram.hpp"

/*--------------------------------------------------------------------*/
/* General Request Properties Telegram Class */
/*--------------------------------------------------------------------*/
class RequestPropertiesTelegram : public ActionTelegram {
	
	public:
	/* Constructor & Destructor */
	RequestPropertiesTelegram();
	RequestPropertiesTelegram( unsigned char _header, short _serviceID);
	~RequestPropertiesTelegram();
	
	/* Memberfunctions */
	short getServiceID();
	
	private:
	/* Attributes */
	short serviceID_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & boost::serialization::base_object<ActionTelegram>(*this); // serialize base class information
        ar & serviceID_;
    }
};

BOOST_CLASS_IMPLEMENTATION(RequestPropertiesTelegram, boost::serialization::object_serializable);


/*--------------------------------------------------------------------*/
/* General Response Properties Telegram Class */
/*--------------------------------------------------------------------*/
class ResponsePropertiesTelegram : public ActionTelegram {
	
	public:
	/* Constructor & Destructor */
	ResponsePropertiesTelegram();
	~ResponsePropertiesTelegram();
	
	/* Memberfunctions */
	short getTarget();
	unsigned char getError();
	
	private:
	/* Attributes */
	short target_;
	unsigned char error_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & boost::serialization::base_object<ActionTelegram>(*this); // serialize base class information
        ar & target_;
        ar & error_;
    }
};

BOOST_CLASS_IMPLEMENTATION(ResponsePropertiesTelegram, boost::serialization::object_serializable);


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
class IOCardPropertiesTelegram : public ResponsePropertiesTelegram {
	
	public:
	/* Constructor & Destructor */
	IOCardPropertiesTelegram();
	~IOCardPropertiesTelegram();
	
	/* Memberfunctions */
	int getNumberAnalogInputs();
	int getNumberAnalogOutpts();
	int getNumberDigitalInputs();
	int getNumberDigitalOutputs();
	
	private:
	/* Attributes */
	int numberAnalogInputs_;
	int numberAnalogOutputs_;
	int numberDigitalInputs_;
	int numberDigitalOutputs_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & boost::serialization::base_object<ResponsePropertiesTelegram>(*this); // serialize base class information
        ar & numberAnalogInputs_;
        ar & numberAnalogOutputs_;
        ar & numberDigitalInputs_;
        ar & numberDigitalOutputs_;
    }
};

BOOST_CLASS_IMPLEMENTATION(IOCardPropertiesTelegram, boost::serialization::object_serializable);


/*--------------------------------------------------------------------*/
/* Telemeter Properties Telegram Class */
/*--------------------------------------------------------------------*/
class TelemeterPropertiesTelegram : public ResponsePropertiesTelegram {


	public:
	/* Constructor & Destructor */
	TelemeterPropertiesTelegram();
	~TelemeterPropertiesTelegram();

	/* Memberfunctions */
	int getNumberTelemeterServices();
	float getX(int teleServiceNo);
	float getY(int teleServiceNo);
	float getT(int teleServiceNo);
	float getFoV(int teleServiceNo);
	float getminD(int teleServiceNo);
	float getmaxD(int teleServiceNo);
	float getdist(int teleServiceNo);

	private:
	/* Attributes */
	int numberOfTelemeterServices_;
	float X_[2];
	float Y_[2];
	float T_[2];
	float FoV_[2];
	float minD_[2];
	float maxD_[2];
	float dist_[2];
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & boost::serialization::base_object<ResponsePropertiesTelegram>(*this); // serialize base class information
		ar & numberOfTelemeterServices_;
		ar & X_[0];
		ar & Y_[0];
		ar & T_[0];
		ar & FoV_[0];
		ar & minD_[0];
		ar & maxD_[0];
		ar & dist_[0];
		ar & X_[1];
		ar & Y_[1];
		ar & T_[1];
		ar & FoV_[1];
		ar & minD_[1];
		ar & maxD_[1];
		ar & dist_[1];
    }
    
};

BOOST_CLASS_IMPLEMENTATION(TelemeterPropertiesTelegram, boost::serialization::object_serializable);

/*--------------------------------------------------------------------*/
/* Battery Properties Telegram Class */
/*--------------------------------------------------------------------*/
class BatteryPropertiesTelegram : public ResponsePropertiesTelegram {
	
	public:
	/* Constructor & Destructor */
	BatteryPropertiesTelegram();
	~BatteryPropertiesTelegram();
	
	/* Memberfunctions */
	float getNominalVoltage();
	float getEnergy();
	unsigned char getMinimumPowerPercentage();
	
	private:
	/* Attributes */
	float nominalVoltage_;
	float energy_;
	unsigned char minimumPowerPercentage_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & boost::serialization::base_object<ResponsePropertiesTelegram>(*this); // serialize base class information
		ar & nominalVoltage_;
		ar & energy_;
		ar & minimumPowerPercentage_;
    }
};

BOOST_CLASS_IMPLEMENTATION(BatteryPropertiesTelegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* Localization Properties Telegram Class */
/*--------------------------------------------------------------------*/
class LocalizationPropertiesTelegram : public ResponsePropertiesTelegram {
	
	public:
	/* Constructor & Destructor */
	LocalizationPropertiesTelegram();
	~LocalizationPropertiesTelegram();
	
	/* Memberfunctions */
	double getInitialX();
	double getInitialY();
	double getInitialTheta();
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
        ar & boost::serialization::base_object<ResponsePropertiesTelegram>(*this); // serialize base class information
		ar & x_;
		ar & y_;
		ar & theta_;
		ar & state_;
    }
};

BOOST_CLASS_IMPLEMENTATION(LocalizationPropertiesTelegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* Differential Properties Telegram Class */
/*--------------------------------------------------------------------*/
class DifferentialPropertiesTelegram : public ResponsePropertiesTelegram {
	
	public:
	/* Constructor & Destructor */
	DifferentialPropertiesTelegram();
	~DifferentialPropertiesTelegram();
	
	/* Memberfunctions */
	float getTargetLinearSpeed();
	float getTargetAngularSpeed();
	float getCurrentLinearSpeed();
	float getCurrentAngularSpeed();
	float getMaximumLinearSpeed();
	float getMinimumLinearSpeed();
	float getMaximumAngularSpeed();
	float getMinimumAngularSpeed();
	float getMaximumLinearAcceleration();
	float getMinimumLinearAcceleration();
	float getMaximumAngularAcceleration();
	float getMinimumAngularAcceleration();
	float getWidth();
	
	private:
	/* Attributes */
	float targetLinearSpeed_;
	float targetAngularSpeed_;
	float currentLinearSpeed_;
	float currentAngularSpeed_;
	float maximumLinearSpeed_;
	float minimumLinearSpeed_;
	float maximumAngularSpeed_;
	float minimumAngularSpeed_;
	float maximumLinearAcceleration_;
	float minimumLinearAcceleration_;
	float maximumAngularAcceleration_;
	float minimumAngularAcceleration_;
	float width_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & boost::serialization::base_object<ResponsePropertiesTelegram>(*this); // serialize base class information
        ar & targetLinearSpeed_;
        ar & targetAngularSpeed_;
        ar & currentLinearSpeed_;
        ar & currentAngularSpeed_;
        ar & maximumLinearSpeed_;
        ar & minimumLinearSpeed_;
        ar & maximumAngularSpeed_;
        ar & minimumAngularSpeed_;
        ar & maximumLinearAcceleration_;
        ar & minimumLinearAcceleration_;
        ar & maximumAngularAcceleration_;
        ar & minimumAngularAcceleration_;
        ar & width_;
    }
};

BOOST_CLASS_IMPLEMENTATION(DifferentialPropertiesTelegram, boost::serialization::object_serializable);


#endif
