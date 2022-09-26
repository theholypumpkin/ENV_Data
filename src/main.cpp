/* main.cpp */
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <EthernetClient.h>
#include <NTPClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_CCS811.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "epd7in5b.h"
#include "epdif.h"
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <math.h>
//#include <Adafruit_SleepyDog.h> //incompatible with megaAVR architecture
#include <LinkedList.h>
#include <EEPROM.h>  
/*================================================================================================*/
#define RANDOM_SEED_ADC_PIN A1 // NOTE NEVER CONNECT A SENSOR TO THIS PIN
#define CCS_811_INTERRUPT_PIN 15 //TODO: Change but unlikly
#define CCS_811_nWAKE 4 //TODO: Change
#define MCP23S17_CS_PIN 6 //SPI-Shiftregister //TODO: Change
#define MCP3002_CS_PIN 8 //SPI-ADC //TODO: Change
#define W5500_CS_PIN 9 //TODO: Change
#define RTC_CLK_PIN 10 //TODO: Change
#define RTC_DAT_PIN 11 //TODO: Change
#define RTC_RST_PIN 12 //TODO: Change
#define DHTPIN 5 //TODO: Change
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 20 //TODO: Change
//NOTE E-Paper SPI pins are defined in library (maybe change?)
/*================================================================================================*/
enum statemachine_t
{
    READ_DHT_SENSOR,
    READ_CCS_SENSOR,
    UPDATE_NTP_CLOCK,
    READ_REAL_TIME_CLOCK,
    E_PAPER_DISPLAY_PARTIL_UPDATE,
    E_PAPER_DISPLAY_FULL_UPDATE,
    PUBLISH_MQTT,
    IDLE
};

volatile statemachine_t e_state = IDLE;
/*================================================================================================*/
LinkedList<uint16_t> g_eco2ValuesList; //60*(16+16+16) = Value+next_ptr+pre_pnt 2,8k RAM
LinkedList<uint16_t> g_tvocValuesList; //60*(16+16+16) = Value+next_ptr+pre_pnt 2,8k RAM
LinkedList<float> g_temperatureValuesList; //60*(32+16+16) = Value+next_ptr+pre_pnt 3,8k RAM
LinkedList<float> g_humidityValuesList; //60*(16+16+16) = Value+next_ptr+pre_pnt 3,8k RAM
//NOTE having 60 Values for each linked list would be 10k+ of RAM, which is more than available
// The Change over 10 minutes wouldn't be sufficently noticeable
// Option 1 would be to only add every 10th reading to the list. This would result in ca 2.3k of RAM
// Option 2 run mean on the list every 10 values or have a temp array which stores 10 readings
// and than means those before appending them to the list
uint16_t g_uuid;
volatile bool b_isrFlag = false; //a flag which is flipped inside an isr
const byte MAC_ADDR[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //TODO maybe this has to be changed if I have this address already in the network
/*________________________________________________________________________________________________*/
Epd ePaperDisplay;
ThreeWire myWire(RTC_DAT_PIN, RTC_CLK_PIN, RTC_RST_PIN); // DAT, CLK, RST
RtcDS1302<ThreeWire> rtc(myWire);
DHT tempHmdSensor(DHTPIN, DHTTYPE); //Create the DHT object
Adafruit_CCS811 co2Sensor;
EthernetClient ethClient;
PubSubClient mqttClient(g_mqttServerUrl, g_mqttServerPort, ethClient);
/*================================================================================================*/
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    Ethernet.init(W5500_CS_PIN);
    if(!Ethernet.begin(MAC_ADDR)){
        //TODO EERROR VIA SERIAL OR SOMEWHERE ELSE
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    setupNetworkTime();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    setupEEPROM();
    //TODO begin MCP23s17 and MCP3002
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    * Attach Hardware interrupt BEFORE Setting up the CCS 811 sensor, because of a race condition.
    * It cloud happen, that the pin was already driven sow by the sensor, before we attach the
    * interrupt. When the pin is already low, we can no longer detect a falling edge, hence the ISR
    * would never trigger.
    */
    attachInterrupt(digitalPinToInterrupt(CCS_811_INTERRUPT_PIN), setReadFlagISRCallback, FALLING);
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic engine of CCS811
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    tempHmdSensor.begin();
    if(!co2Sensor.begin()){
        while(1){
            bool state = digitalRead(LED_BUILTIN);
            digitalWrite(LED_BUILTIN, !state);
            delay(500); //When falure blink LED rapidly
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    co2Sensor.enableInterrupt();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //NOTE when message is greater than 256 than we have to call mqtt.setBuffer(bufferSize); 
    mqttClient.setKeepAlive(70); //keep connection alive for 70 seconds
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    ePaperDisplay.Init();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -
    //TODO need a new watchdog!
    //After ~80 seconds of non responsivness, Watchdog will reset the MCU.*/
    //Watchdog.enable(80000);
}
/*________________________________________________________________________________________________*/
/**
 * @brief The main loop.
 * Because the microcontroller has native USB-Serial and we are constantly connected to a power
 * source, we will not sleep, because this would mess with the server software which reads the
 * Serial connection.
 */
void loop() {
    //make static to retain variables even if out of scope.
    static uint16_t eco2Value, tvocValue;
    static float temperatureValue, humidityValue;
    static bool b_ENVDataCorrection;
    static String timeString;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if(b_isrFlag){
        e_state = READ_DHT_SENSOR; //This is safer than setting the vlaue inside the ISR itself
        b_isrFlag = false;
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch (e_state)
    {
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue);
        /* Unnecessary because we don't break but improves readability
        * optimizer will likly remove it anyway
        */
        e_state = READ_CCS_SENSOR;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
        if(b_ENVDataCorrection){
            readCCSSensor(eco2Value, tvocValue, temperatureValue, humidityValue);
        }else{
            readCCSSensor(eco2Value, tvocValue);
        }
        e_state = PUBLISH_MQTT;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case PUBLISH_MQTT:
        while(!mqttClient.connected()){
            /*client_ID, Username, passwd*/
            if(mqttClient.connect(g_name, g_mqttUsername, g_mqttPassword)){
                if(b_ENVDataCorrection){
                    publishMQTT(eco2Value, tvocValue, temperatureValue, humidityValue);
                }else{
                    publishMQTT(eco2Value,tvocValue);
                }
            }else{
                const char *errorLUT[] = {
                    "MQTT_DISCONNECTED",
                    "MQTT_CONNECT_FAILED",
                    "MQTT_CONNECTION_LOST",
                    "MQTT_CONNECTION_TIMEOUT",
                    "MQTT_CONNECTED", 
                    "MQTT_CONNECT_BAD_PROTOCOL", 
                    "MQTT_CONNECT_BAD_CLIENT_ID", 
                    "MQTT_CONNECT_UNAVAILABLE", 
                    "MQTT_CONNECT_BAD_CREDENTIALS", 
                    "MQTT_CONNECT_UNAUTHORIZED"
                    };
                uint8_t errorMsqIndex = mqttClient.state()+4;
                Serial.println(errorLUT[errorMsqIndex]);
                //wait 5 sec and try again, until we are connected or the watchdog resets the mcu.
                delay(5000); 
            }
        }
        mqttClient.loop();
        //Watchdog.reset();
        e_state = READ_REAL_TIME_CLOCK;
        break;
    case READ_REAL_TIME_CLOCK:
        RtcDateTime now = rtc.GetDateTime();
        if(!now.IsValid()){
            //TODO refresh time with NTP time, Notifiy on E-paper that rtc battery is low or power not connected
        }
        else{
            //Get the time to display on epd
            timeString = String(now.Hour()) + ":" + String(now.Minute());
        }
        e_state = E_PAPER_DISPLAY_PARTIL_UPDATE
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case E_PAPER_DISPLAY_PARTIL_UPDATE:
        break;
    case E_PAPER_DISPLAY_FULL_UPDATE:
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case UPDATE_NTP_CLOCK:
        EthernetUDP ntpUDPObject;
        NTPClient ntpClient(ntpUDPObject, g_ntpTimeServerURL);
        ntpClient.begin();
        updateNetworkTime(ntpUDPObject, ntpClient);
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case IDLE:
    default:
    //TODO need new watchdog
        //Watchdog.sleep(); //sleep until we woken up by interrupt?
        break;
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief Set the Read Flag ISR Callback
 * 
 */
void setReadFlagISRCallback(){
    b_isrFlag = true;
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sets Up the DS1302 RTC with the time fetched from the NTP server when possible, else
 * it sets it to the compiled time 
 */
void setupNetworkTime(){
    rtc.Begin();
    EthernetUDP ntpUDPObject;
    NTPClient ntpClient(ntpUDPObject, g_ntpTimeServerURL);
    ntpClient.begin();
    //If we cant establish a connection to the server
    if(!rtc.IsDateTimeValid()){
        if(!ntpClient.update()){
            RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
            rtc.SetDateTime(compiled);
        }
        else
            updateNetworkTime(ntpUDPObject, ntpClient);
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    if (rtc.GetIsWriteProtected())
        rtc.SetIsWriteProtected(false);

    if (!rtc.GetIsRunning())
        rtc.SetIsRunning(true);
    /* Because we update the RTC at least daily, we don't set a new time in setup exect if the 
     * date is invalid
     */
}
/*________________________________________________________________________________________________*/
/**
 * @brief updates the RTC with the current Network time on a regular basis to stay in sync.
 */
void updateNetworkTime(EthernetUDP ntpUDPObject, NTPClient ntpClient){
    //If we cant establish a connection to the server
    if(ntpClient.update()){
        unsigned long epochTime = ntpClient.getEpochTime() - 946684800; //substract 30 years
        RtcDateTime network = RtcDateTime(epochTime);
        rtc.SetDateTime(network);
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief writes a random UUID to EEPROM (on SAMD chips to Flash) if never set, else it reads the
 * existing UUID from the EEPROM (or on SAMD chips to Flash). If in the first 5 seconds of the setup
 * a button is pressed, it will clear the flash and generate a new UUID
 */
void setupEEPROM(){
    //BUG TEMPORORAY COMMENTED OUT
    /*Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN);
    eepromClearButton.begin();*/
    unsigned long loopEnd = millis() + 5000;
    while(millis() < loopEnd){ //check for 5 seconds if the button is pressed
        //BUG TEMPORORAY COMMENTED OUT
        /*eepromClearButton.read();
        if(eepromClearButton.isPressed()){
            // This loop will take about 3.3*256 ms to complete which is about 0.85 seconds.
            for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
                EEPROM.write(i, 0);
            }
            break; //when we reset the eeprom terminate the while loop
        }*/
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /* For compability reasons and portability to other microcontroller we will not use the more
     * convinient EEPROM.get(addr, g_uuid); and EEPROM.put(addr, g_uuid); methods. 
     * Instead we read individual bytes in the little endian order as it is standard on the AVR 
     * and common on the ARM Architecture.
     */
    uint16_t addr = 0;
    uint16_t uuidUpperByte = (uint16_t) EEPROM.read(addr + 1) << 8;
    uint16_t uuidLowerByte = (uint16_t) EEPROM.read(addr);
    g_uuid = uuidUpperByte + uuidLowerByte; //leftshift by 8 bit
    /* Under the curcumstance that we had reset the eeprom once all bytes are 0.
     * When we never wrote anything to the EEPROM of the microcontroller all bytes will be FF.
     * Because we have a two byte variable, we have to check of the value not 0 or FFFF
     * 
     * We only generate a random variable between 0x100 and 0xFFFE, to we always use both bytes,
     * but never use the "never written" value of FFFF.
     */
    if((g_uuid == 0x0) || (g_uuid == 0xFFFF)){
        //When we have to generate a new UUID generate new seed for Random function
        randomSeed(analogRead(RANDOM_SEED_ADC_PIN));
        g_uuid = (uint16_t)random(0x100,0xFFFE); //generates random uuid in range of 0x100 to 0xFFFE
        uuidUpperByte = g_uuid >> 8;
        uuidLowerByte = (g_uuid << 8) >> 8; //remove the upper 8 bit
        //EEPROM.put(addr, g_uuid);
        //Write the generated UUID to EEPROM
        EEPROM.write(addr, (uint8_t) uuidLowerByte);
        EEPROM.write(addr + 1, (uint8_t) uuidUpperByte);
    }
}
/*________________________________________________________________________________________________*/
/**
 * @brief reads out the CCS 811 metaloxid gas sensor
 * 
 * @param eco2Value A Reference where the eco2 Value should be saved at
 * @param tvocValue A Reference where the tvoc Value should be saved at
 */
void readCCSSensor(uint16_t &eco2Value, uint16_t &tvocValue){
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic Engine of co2Sensor
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    if(!co2Sensor.readData()){
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine od co2_sensor to save power
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
    float temperatureValue, float humidityValue){
    digitalWrite(CCS_811_nWAKE, LOW); //Enable Logic Engine of co2Sensor
    delayMicroseconds(55); // Time until active after nWAKE asserted = 50 us
    co2Sensor.setEnvironmentalData(humidityValue, temperatureValue);
    if(!co2Sensor.readData()){
        eco2Value = co2Sensor.geteCO2();
        tvocValue = co2Sensor.getTVOC();
    }
    delayMicroseconds(25); //Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); //Disable Logic Engine od co2_sensor to save power
}
/*________________________________________________________________________________________________*/
/**
 * @brief Gets the Current Temperature and Humidity
 * 
 * @param temperatureValue A Reference where the temperature Value should be saved at
 * @param humidityValue A Reference where the humidity Value should be saved at
 * @return true if the Reading was sucessful
 * @return false if the reading was unsucessful and the read value is "Not A Number"
 */
bool readDHTSensor(float &temperatureValue, float &humidityValue){
    temperatureValue = tempHmdSensor.readTemperature();
    humidityValue = tempHmdSensor.readHumidity();
    if (isnan(humidityValue) || isnan(temperatureValue)) {
        return false;
    }
    return true;
} 
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 * 
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param dustDensityValue The read Dust Density Value
 * @param dustSensorBaseline The dust Sensor baseline when available
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue){
    
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    //using a buffer speeds up the mqtt publishing process by over 100x
    char mqttJsonBuffer[100];
    size_t n = serializeJson(json, mqttJsonBuffer); //saves a bit of time when publishing
    mqttClient.publish(g_indoorAirQualityTopic, mqttJsonBuffer, n);
}
/*________________________________________________________________________________________________*/
/**
 * @brief Sends the Read Sensor values via the Serial interface to the Server to be saved in the
 * database.
 * 
 * @param eco2Value The read CO2 Value
 * @param tvocValue The read TVOC Value
 * @param dustDensityValue The read Dust Density Value
 * @param temperatureValue The read Temperature Value
 * @param humidityValue The Read Humidity Value
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue,
    float temperatureValue, float humidityValue){
    StaticJsonDocument<100> json; //create a json object //NOTE size of document check
    //json["measurement"].set(g_influxDbMeasurement);
    json["tags"]["location"].set(g_location);
    json["tags"]["uuid"].set(g_uuid);
    json["tags"]["name"].set(g_name);
    json["fields"]["eCO2"].set(eco2Value);
    json["fields"]["TVOC"].set(tvocValue);
    json["fields"]["temperature"].set(temperatureValue);
    json["fields"]["humidity"].set(humidityValue);
    //using a buffer speeds up the mqtt publishing process by over 100x
    char mqttJsonBuffer[100];
    size_t n = serializeJson(json, mqttJsonBuffer);
    mqttClient.publish(g_indoorAirQualityTopic, mqttJsonBuffer, n);
}
/*end of file*/