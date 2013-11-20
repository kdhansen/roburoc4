#ifndef TELEGRAM_HPP
#define TELEGRAM_HPP

#include <boost/serialization/base_object.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

/* Telefram header defines */
#define PURE_HEADER_STATUS		0xFF
#define PURE_HEADER_ERROR		0x00

/* Service ID Defines */
#define PURE_SERVICE_ID_DIRECTORY	0x0000
#define PURE_SERVICE_ID_NOTIFICATION	0x0001
#define PURE_SERVICE_ID_IOCARD		0x0002
#define PURE_SERVICE_ID_TELEMETER	0x0003
#define PURE_SERVICE_ID_BATTERY		0x0004
#define PURE_SERVICE_ID_LOCALIZATION	0x0005
#define PURE_SERVICE_ID_DIFFERENTIAL	0x0006

/* Action defines */
#define PURE_ACTION_GET			0x00
#define PURE_ACTION_QUERY		0x01
#define PURE_ACTION_REPLACE		0x02
#define	PURE_ACTION_INSERT		0x04


/*--------------------------------------------------------------------*/
/* General Telegram Class */
/*--------------------------------------------------------------------*/
class Telegram {

	public:
	/* Constructor & Destructor */
	Telegram();
	Telegram( unsigned char _header );
	~Telegram();
	
	/* Memberfunctions */
	unsigned char getHeader();
			
	protected:
	/* Attributes */
	unsigned char header_;
	
	private:
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & header_;
    }
};

BOOST_CLASS_IMPLEMENTATION(Telegram, boost::serialization::object_serializable);



/*--------------------------------------------------------------------*/
/* Action Telegram class */
/*--------------------------------------------------------------------*/
class ActionTelegram : public Telegram {
	
	public:
	/* Constructor & Destructor */
	ActionTelegram();
	ActionTelegram( unsigned char _header, unsigned char _action);
	~ActionTelegram();
	
	/* Memberfunctions */
	unsigned char getAction();
	
	protected:
	/* Attributes */
	unsigned char action_;
	
	private:
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<Telegram>(*this);
        ar & action_;
    }
};

BOOST_CLASS_IMPLEMENTATION(ActionTelegram, boost::serialization::object_serializable);

















#endif
