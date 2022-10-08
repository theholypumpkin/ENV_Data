/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include <ArduinoJson.h>
#include <JC_Button.h>
#include <PubSubClient.h>
#include <math.h>
#include <RTCZero.h> //Also inplements sleep
/* SAMD21. SAMD51 chips do not have EEPROM. This library provides an EEPROM-like API and hence
 * allows the same code to be used.
 */
#include <FlashAsEEPROM.h>
/*================================================================================================*/
#define RANDOM_SEED_ADC_PIN A0 // NOTE: NEVER CONNECT A SENSOR TO THIS PIN
#define BATTERY_VOLTAGE_ADC_PIN A1
#define CCS_811_INTERRUPT_PIN 13
#define CCS_811_nWAKE 16
#define DHTPIN 21
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 20
#define DOCUMENT_SIZE 255
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    READ_BATTERY,
    PUBLISH_MQTT,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
uint8_t g_lastRtcUpdateDay;
uint16_t g_uuid = 47950; //BUG REMOVE STATIC
//We use two 9V block batteries to keep current low
const float MAX_BATTERY_VOLTAGE = 18.0, //use a R1 = 10k und R2 = 2k Voltage Divider
            ADC_VOLTAGE_FACTOR = MAX_BATTERY_VOLTAGE / powf(2.0, ADC_RESOLUTION);

volatile bool b_isrFlag = false; // a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
RTCZero rtc;
DHT tempHmdSensor(DHTPIN, DHTTYPE); // Create the DHT object
Adafruit_CCS811 co2Sensor;
WiFiClient wifiClient;
PubSubClient mqttClient(g_mqttServerUrl, g_mqttServerPort, wifiClient);
/*================================================================================================*/
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    analogReadResolution(ADC_RESOLUTION);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //setupEEPROM(); //BUG RE-ENABLE
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    Serial.begin(9600);            // Use Hardware Serial (not USB Serial) to debug
    Serial.println("Setup");
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic engine of CCS811
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if (!co2Sensor.begin())
    {
        while (1)
        {
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); // When falure blink LED rapidly
            Serial.println("CCS Error");
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Begin the Wifi Connection
    Serial.println("Connecting...");
    WiFi.setHostname(g_name);
    //esablishing wifi connection
    while(WiFi.begin(g_wifiSsid, g_wifiPass) != WL_CONNECTED){ //retry connect indef
        delay(100);
        //TODO Wire gpio to reset pin and triger it after some time
    }
    WiFi.lowPowerMode();
    Serial.println("WiFi Connected");
    //Serial.print("IP Address: ");
    //Serial.println(WiFi.localIP());
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    mqttClient.setKeepAlive(70); //Keep Connection alive for 70 seconds
    mqttClient.setBufferSize(DOCUMENT_SIZE);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    rtc.begin(); //begin the rtc at "random time" probably  Jan 1 2000 at 00:00:00 o'clock
    g_lastRtcUpdateDay = rtc.getDay();
    if(updateNetworkTime()){ //set real time acording to network
        Serial.println("Network Time successful");
        /*Serial.print(rtc.getHours());
        Serial.print(":");
        Serial.print(rtc.getMinutes());
        Serial.print(":");
        Serial.print(rtc.getSeconds());
        Serial.print(" ");
        Serial.print(rtc.getDay());
        Serial.print("/");
        Serial.print(rtc.getMonth());
        Serial.print("/");
        Serial.println(rtc.getYear()); */ 
    }else{
        Serial.println("ERROR: No Network Time");
    }
    //rtc.setAlarmTime(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    //rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear());
    rtc.setAlarmSeconds(rtc.getSeconds()+10);
    rtc.enableAlarm(rtc.MATCH_SS); //Set Alarm every minute
    rtc.attachInterrupt(alarmISRCallback); //When alarm trigger this callback.
    /*Serial.print("Alarm Time: ");
    Serial.print(rtc.getAlarmHours());
    Serial.print(":");
    Serial.print(rtc.getAlarmMinutes());
    Serial.print(":");
    Serial.print(rtc.getAlarmSeconds());
    Serial.print(" ");
    Serial.print(rtc.getAlarmDay());
    Serial.print("/");
    Serial.print(rtc.getAlarmMonth());
    Serial.print("/");
    Serial.println(rtc.getAlarmYear());*/
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delay(10000); // give us some time to upload a new program
}
/*________________________________________________________________________________________________*/
/**
 * @brief The main loop.
 * Because the microcontroller has native USB-Serial and we are constantly connected to a power
 * source, we will not sleep, because this would mess with the server software which reads the
 * Serial connection.
 */
void loop()
{
    // make static to retain variables even if out of scope.
    static uint16_t eco2Value, tvocValue;
    static float temperatureValue, humidityValue, heatIndexValue, batteryPercentage, batteryVoltage;
    static bool b_ENVDataCorrection;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if (b_isrFlag)
    {
        e_state = READ_DHT_SENSOR; // This is safer than setting the vlaue inside the ISR itself
        b_isrFlag = false;
    }
    if(g_lastRtcUpdateDay != rtc.getDay()){ //update network time daily
        if(updateNetworkTime()){ //set real time acording to network
            Serial.println("Network Time successful");
        }else{
            Serial.println("ERROR: No Network Time");
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch(e_state){
    case IDLE:
        //Go to sleep until Alarm wakes you
        //Serial.println("Going to sleep");
        //rtc.standbyMode();
        /*for (int i = 0; i < 5; i++){
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500);
        }
        digitalWrite(LED_BUILTIN, LOW);*/
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_DHT_SENSOR:
        //Serial.println("State: DHT");
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue, heatIndexValue);
        e_state = READ_CCS_SENSOR;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        //Serial.println("State: CCS");
        if (b_ENVDataCorrection)
        {
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }
        else
        {
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = READ_BATTERY;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_BATTERY:
        //Serial.println("State: BATTERY");
        batteryVoltage = analogRead(BATTERY_VOLTAGE_ADC_PIN) * ADC_VOLTAGE_FACTOR;
        batteryPercentage = calcBatteryPercentageLiPo(batteryVoltage);
        e_state = PUBLISH_MQTT;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case PUBLISH_MQTT:
        //Serial.println("State: MQTT");
        mqttReconnect();
        long wifiSignalStrength = WiFi.RSSI();
        if (b_ENVDataCorrection)
        {
            publishMQTT(eco2Value, tvocValue, wifiSignalStrength, temperatureValue, 
            humidityValue, heatIndexValue, batteryVoltage, batteryPercentage);
        }
        else
        {
            publishMQTT(eco2Value, tvocValue, wifiSignalStrength, batteryVoltage, 
            batteryPercentage);
        }
        mqttClient.loop();
        e_state = IDLE;
        break;
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Set the Read Flag ISR Callback
 */
void alarmISRCallback()
{
    b_isrFlag = true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Connects to WiFi using a static IP-Address, DNS, Gateway and Subnet than
 * it fetchs the latest network time and set the Real Time Clock to this time.
 * Afterwards it disconnects from wifi to conserve battery.
 */
bool updateNetworkTime(){
    WiFiUDP ntpUdpObject;
    NTPClient ntpClient(ntpUdpObject, g_ntpTimeServerURL, 7200);
    ntpClient.begin();
    uint8_t connection_attempts = 0;
    while(!ntpClient.update()){ //attempt to connect to ntp server up to 5 times.
        connection_attempts++;
        if(connection_attempts == 4){
            return false;
        }
        delay(1000);
    }
    unsigned long epochTime = ntpClient.getEpochTime();
    rtc.setEpoch(epochTime);
    g_lastRtcUpdateDay = rtc.getDay(); //set to actual day
    return true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Attempt to reconnect to mqtt Server
 * 
 */
void mqttReconnect() {
  // Loop until we're reconnected
    while (!mqttClient.connected()) {
        if (!mqttClient.connect(g_name, g_mqttUsername, g_mqttPassword)){
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 1 seconds");
            // Wait 1 seconds before retrying
            delay(1000);
        }
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief writes a random UUID to EEPROM (on SAMD chips to Flash) if never set, else it reads the
 * existing UUID from the EEPROM (or on SAMD chips to Flash). If in the first 5 seconds of the setup
 * a button is pressed, it will clear the flash and generate a new UUID
 */
void setupEEPROM()
{

    Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN);
    eepromClearButton.begin();
    unsigned long loopEnd = millis() + 5000;
    while (millis() < loopEnd)
    { // check for 5 seconds if the button is pressed
        eepromClearButton.read();
        if (eepromClearButton.isPressed())
        {
            // This loop will take about 3.3*256 ms to complete which is about 0.85 seconds.
            for (uint16_t i = 0; i < EEPROM.length(); i++)
            {
                EEPROM.write(i, 0);
            }
            break; // when we reset the eeprom terminate the while loop
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /* For compability reasons and portability to other microcontroller we will not use the more
     * convinient EEPROM.get(addr, g_uuid); and EEPROM.put(addr, g_uuid); methods.
     * Instead we read individual bytes in the little endian order as it is standard on the AVR
     * and common on the ARM Architecture.
     */
    uint16_t addr = 0;
    uint16_t uuidUpperByte = (uint16_t)EEPROM.read(addr + 1) << 8;
    uint16_t uuidLowerByte = (uint16_t)EEPROM.read(addr);
    g_uuid = uuidUpperByte + uuidLowerByte; // leftshift by 8 bit
    /* Under the curcumstance that we had reset the eeprom once all bytes are 0.
     * When we never wrote anything to the EEPROM of the microcontroller all bytes will be FF.
     * Because we have a two byte variable, we have to check of the value not 0 or FFFF
     *
     * We only generate a random variable between 0x100 and 0xFFFE, to we always use both bytes,
     * but never use the "never written" value of FFFF.
     */
    if ((g_uuid == 0x0) || (g_uuid == 0xFFFF))
    {
        // When we have to generate a new UUID generate new seed for Random function
        randomSeed(analogRead(RANDOM_SEED_ADC_PIN));
        g_uuid = (uint16_t)random(0x100, 0xFFFE); // generates random uuid in range of 0x100 to 0xFFFE
        uuidUpperByte = g_uuid >> 8;
        uuidLowerByte = (g_uuid << 8) >> 8; // remove the upper 8 bit
        // EEPROM.put(addr, g_uuid);
        // Write the generated UUID to EEPROM
        EEPROM.write(addr, (uint8_t)uuidLowerByte);
        EEPROM.write(addr + 1, (uint8_t)uuidUpperByte);
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 *
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the tvoc Value should be saved at
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue)
{
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic Engine of co2Sensor
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    /*uint8_t before = rtc.getSeconds();
    while(!co2Sensor.available()){
        delay(1000);
        Serial.println("Waiting fro CCS Data");
        } //due to clock drifing we loop until data is available
    uint8_t after = rtc.getSeconds();
    Serial.println("Data available");*/
    if (!co2Sensor.readData())
    {
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 *
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the eco2 Value should be saved at
 * @param temperatureValue The current temperature to create a more accurate reading
 * @param humidityValue The current humidity to create a more accurate reading
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue,
                   float temperatureValue, float humidityValue)
{
    digitalWrite(CCS_811_nWAKE, LOW); // Enable Logic Engine of co2Sensor
    delayMicroseconds(55);            // Time until active after nWAKE asserted = 50 us
    co2Sensor.setEnvironmentalData(humidityValue, temperatureValue);
    /*uint8_t before = rtc.getSeconds();
    while(!co2Sensor.available()){
        delay(1000);
        Serial.println("Waiting fro CCS Data");
        } //due to clock drifing we loop until data is available
    uint8_t after = rtc.getSeconds();
    Serial.println("Data available");*/
    if (!co2Sensor.readData())
    {
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine od co2_sensor to save power
    /* If we idle very long because the CO2 Sensor internal clock and the alarm clock are out of
     * sync, we set the alarm to the closest second, so we can maximize sleep
     */
    //if (before != after){ //When we idle very long
        //rtc.detachInterrupt();
        //rtc.disableAlarm();
    //    rtc.setAlarmSeconds(after);
        //rtc.enableAlarm(rtc.MATCH_SS);
        //rtc.attachInterrupt(alarmISRCallback);
    //}
}
/*________________________________________________________________________________________________*/
/**
 * @brief Gets the Current Temperature and Humidity
 *
 * @param temperatureValue A Reference where the temperature Value should be saved at
 * @param humidityValue A Reference where the humidity Value should be saved at
 * @param heatIndexValue A Reference where the heat index Value should be saved at
 * @return true if the Reading was sucessful
 * @return false if the reading was unsucessful and the read value is "Not A Number"
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue, float &heatIndexValue)
{
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue))
    {
        return false;
    }
    heatIndexValue = tempHmdSensor.computeHeatIndex(temperatureValue, humidityValue, false);
    return true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 *
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param rssi The WiFi Signal Strength
 * @param voltage the battery voltage
 * @param percantage the calculated battery percentage
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssi, 
float voltage, float percentage)
{

    StaticJsonDocument<DOCUMENT_SIZE> json; // create a json object //NOTE size of document check
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["Battery Voltage"].set(voltage);
    json["fields"]["Battery Percentage"].set(percentage);
    json["fields"]["WiFi RSSI"].set(rssi);
    // using a buffer speeds up the mqtt publishing process by over 100x
    char mqttJsonBuffer[DOCUMENT_SIZE];
    size_t n = serializeJson(json, mqttJsonBuffer); // saves a bit of time when publishing
    bool success  = mqttClient.publish(g_indoorAirQualityTopic, mqttJsonBuffer, n);
    Serial.print("Publish successful? ");
    Serial.println(success ? "Yes" : "No");
    Serial.println(mqttJsonBuffer);
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 *
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param rssi The WiFi Signal Strength
 * @param temperatureValue The read Temperature Value
 * @param humidityValue The read Humidity Value
 * @param heatIndexValue The calculated heat index value
 * @param voltage the battery voltage
 * @param percantage the calculated battery percentage
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssi,
                 float temperatureValue, float humidityValue, float heatIndexValue, 
                 float voltage, float percentage)
{
    StaticJsonDocument<DOCUMENT_SIZE> json; // create a json object //NOTE size of document check
    // json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["temperature"].set(temperatureValue);
    json["fields"]["humidity"].set(humidityValue);
    json["fields"]["heat index"].set(heatIndexValue);
    json["fields"]["Battery Voltage"].set(voltage);
    json["fields"]["Battery Percentage"].set(percentage);
    json["fields"]["WiFi RSSI"].set(rssi);
    // using a buffer speeds up the mqtt publishing process by over 100x
    char mqttJsonBuffer[DOCUMENT_SIZE];
    size_t n = serializeJson(json, mqttJsonBuffer);
    bool success  = mqttClient.publish(g_indoorAirQualityTopic, mqttJsonBuffer, n);
    Serial.print("Publish successful? ");
    Serial.println(success ? "Yes" : "No");
    Serial.print("Reason: ");
    char *arr[10] = {"MQTT_CONNECTION_TIMEOUT", "MQTT_CONNECTION_LOST", "MQTT_CONNECT_FAILED", 
                    "MQTT_DISCONNECTED", "MQTT_CONNECTED", "MQTT_CONNECT_BAD_PROTOCOL", 
                    "MQTT_CONNECT_BAD_CLIENT_ID", "MQTT_CONNECT_UNAVAILABLE", 
                    "MQTT_CONNECT_BAD_CREDENTIALS", "MQTT_CONNECT_UNAUTHORIZED"};
    Serial.println(arr[mqttClient.state()+4]);
    Serial.println(mqttJsonBuffer);
}
/*________________________________________________________________________________________________*/
/**
 * @brief Calculate the battery percentage acording to two differnet formulas depending on the
 * voltage read by the ADC.
 *
 * @param adcValue the integer Reading of the ADC across the voltage divider. The factor to convert
 * the value is calculated only once on setup.
 * @return float the battery percantage based on the formula y = 120x-404 for change above
 * 63% and 255x-930 for charge below 63%.
 */
float calcBatteryPercentageLiPo(float x)
{
    if (x < 3.896)
        return 255.0f*x - 930.0f;
    else if (x < 3.648)
        return 0.0;
    else
        return 120.0f*x-404;
}
//TODO meassure (9V block lithum battery discharge rate and generate formula)
/*end of file*/