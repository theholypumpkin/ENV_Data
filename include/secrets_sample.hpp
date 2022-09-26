#ifndef Arduino_h
    #include <Arduino.h>
#endif
#ifndef _SECRETS_HPP
#define _SECRETS_HPP
extern const char* g_name = "any name you like";
extern const char* g_location = "a location";
extern const IPAddress g_mqttServerUrl(0,0,0,0); //TODO change to url
extern const uint16_t g_mqttServerPort = 8883;
extern const char* g_mqttUsername = "your username";
extern const char* g_mqttPassword = "your password";
extern const char* g_ntpTimeServerURL = "timeserver.a-url.com";

extern const char* g_indoorAirQualityTopic = "data/topic/";
#endif