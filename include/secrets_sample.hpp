#ifndef Arduino_h
    #include <Arduino.h>
#endif
#ifndef _SECRETS_HPP
#define _SECRETS_HPP
extern const char* g_name = "a name you like";
extern const char* g_location = "kitchen or whatever";
extern const IPAddress g_staticIPAddress(0,0,0,0);
extern const IPAddress g_DNSAddress(1,1,1,1);
extern const IPAddress g_gateway(0,0,0,0);
extern const IPAddress g_subnet(255,255,255,0);
extern const IPAddress g_mqttServerUrl(0,0,0,0);
extern const uint16_t g_mqttServerPort = 8883;
extern const char* g_mqttUsername = "your usename";
extern const char* g_mqttPassword = "your password";
extern const char* g_wifiSsid = "wifi name";
extern const char* g_wifiPass = "wifi password";

extern const char* g_indoorAirQualityTopic = "data/topic/";
#endif