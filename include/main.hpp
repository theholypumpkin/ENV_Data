#ifndef Arduino_h
    #include <Arduino.h>
#endif 
#ifndef _MAIN_HPP
#define _MAIN_HPP
void setup();
void loop();
void setReadFlagISRCallback();
void setupEEPROM();
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue);
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue, 
    float temperatureValue, float humidityValue);
bool readDHTSensor(float &temperatureValue, float &humidityValue);
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue,
    float temperatureValue, float humidityValue);
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue);
void setupNetworkTime();
void updateNetworkTime(EthernetUDP ntpUDPObject, NTPClient ntpClient);
#endif