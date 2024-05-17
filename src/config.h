#pragma once

#include <string>

class Config
{
public:
	Config();
	bool load(const std::string file_name);

	std::string felinet_address() { return m_felinet_address; }
	std::string callsign() { return m_callsign; }
	std::string comment() { return m_comment; }
	int ssid() { return m_ssid; }
	int icon() { return m_icon; } 
	int beacon_interval() { return m_beacon_interval; }
	int altitude() { return m_altitude; }
	int antenna_height() { return m_antenna_height; }
	double latitude() { return m_latitude; }
	double longitude() { return m_longitude; }
	float antenna_gain() { return m_antenna_gain; }

	void set_felinet_address(const std::string address) { m_felinet_address = address; }
	void set_callsign(const std::string callsign) { m_callsign = callsign; }
	void set_comment(const std::string comment) { m_comment = comment; }
	void set_ssid(const int ssid) { m_ssid = ssid; }
	void set_icon(const int icon) { m_icon = icon; }
	void set_beacon_interval(const int beacon_interval) { m_beacon_interval = beacon_interval; }
	void set_altitude(const int altitude) { m_altitude = altitude; }
	void set_antenna_height(const int antenna_height) { m_antenna_height = antenna_height; }
	void set_latitude(const double latitude) { m_latitude = latitude; }
	void set_longitude(const double longitude) { m_longitude = longitude; }
	void set_antenna_gain(const float antenna_gain) { m_antenna_gain = antenna_gain; }

private:
	std::string m_felinet_address;
	std::string m_callsign;
	std::string m_comment;
	int m_ssid;
	int m_icon;
	int m_beacon_interval;
	int m_altitude;
	int m_antenna_height;
	double m_latitude;
	double m_longitude;
	float m_antenna_gain;
};
