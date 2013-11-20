#ifndef REPLACE_TELEGRAM_HPP
#define REPLACE_TELEGRAM_HPP

#include <telegram.hpp>

/*--------------------------------------------------------------------*/
/* Localization Replace Telegram class */
/*--------------------------------------------------------------------*/
class LocalizationReplaceTelegram : public ActionTelegram {
	
	public:
	/* Constructor & Destructor */
	LocalizationReplaceTelegram();
	LocalizationReplaceTelegram( unsigned char _header, double _x, double _y, double theta);
	~LocalizationReplaceTelegram();
	
	private:
	/* Attributes */
	short target_;
	unsigned long int timeStamp_;
	double x_;
	double y_;
	double theta_;
	
	/* Serialization */
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        // serialize base class information
        ar & boost::serialization::base_object<ActionTelegram>(*this);
        ar & target_;
        ar & timeStamp_;
        ar & x_;
        ar & y_;
        ar & theta_;
    }
};

BOOST_CLASS_IMPLEMENTATION(LocalizationReplaceTelegram, boost::serialization::object_serializable);

#endif
