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
//#include <ArduinoJson.h>
#include <JC_Button.h>
//#include <PubSubClient.h>
#include <ArduinoHA.h>
#include <math.h>
#include <RTCZero.h> //Also inplements sleep
/* SAMD21. SAMD51 chips do not have EEPROM. This library provides an EEPROM-like API and hence
 * allows the same code to be used.
 */
#ifdef ARDUINO_SAMD_NANO_33_IOT
#include <FlashAsEEPROM.h>
#endif
/*================================================================================================*/
#define RANDOM_SEED_ADC_PIN A2 //NEVER CONNECT A SENSOR TO THIS PIN
#define BATTERY_VOLTAGE_ADC_PIN A6
#define MAX_1676_LOW_BATTERY_OUT_INTERRUPT_PIN 21 // LBO PIN (A7)
#define CCS_811_INTERRUPT_PIN 15 //A1
#define CCS_811_nWAKE 14 //A0
#define DHTPIN 7
#define DHTTYPE DHT22
#define EEPROM_CLEAR_BUTTON_PIN 8
#define EEPROM_EMULATION_SIZE 64 //We really only need 2 bytes but it still less the the default 1k.
#define DOCUMENT_SIZE 511 //If publishing fails, increase the Document Size
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
uint16_t g_uuid;
//We use two 9V block batteries to keep current low
const float MAX_BATTERY_VOLTAGE = 4.2, //use a R1 = 1k, 3.3k or or higher in same ratio.
            ADC_VOLTAGE_FACTOR = MAX_BATTERY_VOLTAGE / powf(2.0, ADC_RESOLUTION);

volatile bool b_isrFlag = false; // a flag which is flipped inside an isr
/*________________________________________________________________________________________________*/
RTCZero rtc;
DHT tempHmdSensor(DHTPIN, DHTTYPE); // Create the DHT object
Adafruit_CCS811 co2Sensor;
WiFiClient wifiClient;
//PubSubClient mqttClient(g_mqttServerUrl, g_mqttServerPort, wifiClient);
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
HADevice device;
HAMqtt mqttClient(wifiClient, device);
HASensorNumber tempHASensor("temp", HASensorNumber::PrecisionP2);
HASensorNumber hmdHASensor("hmd", HASensorNumber::PrecisionP2);
HASensorNumber heatIndexHASensor("hI", HASensorNumber::PrecisionP2);
HASensorNumber co2HASensor("co2");
HASensorNumber tvocHASensor("aqi");
HASensorNumber batVoltageHASensor("batV", HASensorNumber::PrecisionP2);
HASensorNumber batPercentHASensor("batP", HASensorNumber::PrecisionP2);
HASensorNumber rssiHASensor("rssi");
HASensor locationHASensor("loc");
/*================================================================================================*/
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CCS_811_nWAKE, OUTPUT);
    analogReadResolution(ADC_RESOLUTION);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    Serial1.begin(9600);            // Use Hardware Serial (not USB Serial) to debug
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    setupEEPROM();
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
            Serial1.println("CCS Error");
        }
    }
    co2Sensor.setDriveMode(CCS811_DRIVE_MODE_60SEC);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    delayMicroseconds(25);             // Logic engine should run at least 20 us
    digitalWrite(CCS_811_nWAKE, HIGH); // Disable Logic Engine
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Begin the Wifi Connection
    Serial1.print("Connecting...");
    WiFi.setHostname(g_name);
    //esablishing wifi connection
    while(WiFi.begin(g_wifiSsid, g_wifiPass) != WL_CONNECTED){ //retry connect indef
        delay(100);
        Serial1.print(".");
        //TODO Wire gpio to reset pin and triger it after some time
    }
    WiFi.lowPowerMode();
    Serial1.println("connected");
    Serial1.print("IP Address: ");
    Serial1.println(WiFi.localIP());
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    byte uuid_arr[2] = {
        (byte)g_uuid >> 8, //remove the lower byte and retains the uppper byte
        (byte)(g_uuid << 8) >> 8 //removes the upper byte by shifting it out of range and than back.
        }; 
    
    device.setUniqueId(uuid_arr, 2);
    device.setName(g_name);
    mqttClient.begin(g_mqttServerUrl, g_mqttServerPort, g_mqttUsername, g_mqttPassword);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Setting the parameters of the Sensors in a way Home Assitant understands them
    tempHASensor.setName("Temperature");
    hmdHASensor.setName("Humidity");
    heatIndexHASensor.setName("Heat Index");
    co2HASensor.setName("CO2");
    tvocHASensor.setName("Air Quality");
    batVoltageHASensor.setName("Battery Voltage");
    batPercentHASensor.setName("Battery Percentage");
    rssiHASensor.setName("WiFi Signal Strength");

    tempHASensor.setUnitOfMeasurement("°C");
    hmdHASensor.setUnitOfMeasurement("%");
    heatIndexHASensor.setUnitOfMeasurement("°C");
    co2HASensor.setUnitOfMeasurement("ppm");
    tvocHASensor.setUnitOfMeasurement("ppb");
    batVoltageHASensor.setUnitOfMeasurement("V");
    batPercentHASensor.setUnitOfMeasurement("%");
    rssiHASensor.setUnitOfMeasurement("dBm");

    tempHASensor.setIcon("mdi:temperature-celsius");
    hmdHASensor.setIcon("mdi:water-percent");
    heatIndexHASensor.setIcon("mdi:temperature-celsius");
    co2HASensor.setIcon("mdi:molecule-co2");
    tvocHASensor.setIcon("mdi:air-filter");
    batVoltageHASensor.setIcon("mdi:battery");
    batPercentHASensor.setIcon("mdi:battery");
    rssiHASensor.setIcon("mdi:wifi-strength-3-alert");

    tempHASensor.setDeviceClass("temperature");
    hmdHASensor.setDeviceClass("humidity");
    heatIndexHASensor.setDeviceClass("temperature");
    co2HASensor.setDeviceClass("carbon_dioxide");
    tvocHASensor.setDeviceClass("aqi");
    batVoltageHASensor.setDeviceClass("voltage");
    batPercentHASensor.setDeviceClass("battery");
    rssiHASensor.setDeviceClass("signal_strength");
    
    locationHASensor.setValue(g_location);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    rtc.begin(); //begin the rtc at "random time" probably  Jan 1 2000 at 00:00:00 o'clock
    g_lastRtcUpdateDay = rtc.getDay();
    if(updateNetworkTime()){ //set real time acording to network
        Serial1.println("Network Time successful");
    }else{
        Serial1.println("ERROR: No Network Time");
    }
    rtc.setAlarmSeconds(rtc.getSeconds()-1);
    rtc.enableAlarm(rtc.MATCH_SS); //Set Alarm every minute
    rtc.attachInterrupt(alarmISRCallback); //When alarm trigger this callback.
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //TODO Attach LBO Interrupt
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
        e_state = READ_DHT_SENSOR; // This is safer than setting the value inside the ISR itself
        b_isrFlag = false;
    }
    if(g_lastRtcUpdateDay != rtc.getDay()){ //update network time daily
        if(updateNetworkTime()){ //set real time acording to network
            Serial1.println("Updated Network Time");
        }else{
            Serial1.println("Network Time Update failed");
        }
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch(e_state){
    case IDLE:
        Serial1.println("Sleeping now");
        rtc.standbyMode();
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_DHT_SENSOR:
        b_ENVDataCorrection = readDHTSensor(temperatureValue, humidityValue, heatIndexValue);
        e_state = READ_CCS_SENSOR;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case READ_CCS_SENSOR:
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
        batteryVoltage = analogRead(BATTERY_VOLTAGE_ADC_PIN) * ADC_VOLTAGE_FACTOR;
        batteryPercentage = calcBatteryPercentageLiPo(batteryVoltage);
        e_state = PUBLISH_MQTT;
        break;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    case PUBLISH_MQTT:
        mqttClient.loop();
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
        //mqttClient.loop();
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
 * @brief writes a random UUID to EEPROM (on SAMD chips to Flash) if never set, else it reads the
 * existing UUID from the EEPROM (or on SAMD chips to Flash). If in the first 5 seconds of the setup
 * a button is pressed, it will clear the flash and generate a new UUID
 */
void setupEEPROM()
{
    Serial1.print("Awaiting Button Press");
    //BUG UUID is alway reset it should stay the same!
    Button eepromClearButton(EEPROM_CLEAR_BUTTON_PIN);
    eepromClearButton.begin();
    unsigned long loopEnd = millis() + 5000;
    while (millis() < loopEnd)
    { // check for 5 seconds if the button is pressed
        #ifdef ARDUINO_SAMD_NANO_33_IOT
        if(!EEPROM.isValid()) //If never written to the EEPROM Emulation, ignore the button.
        #endif
            break;
        Serial1.print(".");
        eepromClearButton.read();
        if (eepromClearButton.isPressed())
        {
            Serial.println("Button was pressed");
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
    Serial1.print("Current UUID: ");
    Serial1.println(g_uuid, HEX);
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
        #ifdef ARDUINO_SAMD_NANO_33_IOT
        EEPROM.commit(); //Accually writing the data to the EEPROM Emulation!
        #endif
    }
    Serial1.print("New UUID: ");
    Serial1.println(g_uuid, HEX);
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
 * @param percentage the calculated battery percentage
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssiValue, 
float voltage, float percentage)
{
    co2HASensor.setValue(eco2Value);
    tvocHASensor.setValue(tvocValue);
    rssiHASensor.setValue(rssiValue);
    batVoltageHASensor.setValue(voltage);
    batPercentHASensor.setValue(percentage);

    Serial1.print("eco2: "); Serial1.print(eco2Value);
    Serial1.print("tvoc: "); Serial1.print(tvocValue);
    Serial1.print("rssi: "); Serial1.print(rssiValue);
    Serial1.print("voltage: "); Serial1.print(voltage);
    Serial1.print("percentage: "); Serial1.println(percentage);
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
 * @param percentage the calculated battery percentage
 */
void publishMQTT(uint16_t eco2Value, uint16_t tvocValue, long rssiValue,
                 float temperatureValue, float humidityValue, float heatIndexValue, 
                 float voltage, float percentage)
{
    tempHASensor.setValue(temperatureValue);
    hmdHASensor.setValue(humidityValue);
    heatIndexHASensor.setValue(heatIndexValue);
    co2HASensor.setValue(eco2Value);
    tvocHASensor.setValue(tvocValue);
    rssiHASensor.setValue(rssiValue);
    batVoltageHASensor.setValue(voltage);
    batPercentHASensor.setValue(percentage);

    Serial1.print("temperature: "); Serial1.print(temperatureValue);
    Serial1.print("humidity: "); Serial1.print(humidityValue);
    Serial1.print("heatIndex: "); Serial1.print(heatIndexValue);
    Serial1.print("eco2: "); Serial1.print(eco2Value);
    Serial1.print("tvoc: "); Serial1.print(tvocValue);
    Serial1.print("rssi: "); Serial1.print(rssiValue);
    Serial1.print("voltage: "); Serial1.print(voltage);
    Serial1.print("percentage: "); Serial1.println(percentage);
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
    if(x > 3.896)
        return 120.0f*x-404;
    else if (x > 3.648)
        return 255.0f*x - 930.0f;      
    else
        return 0.0;
}
/*end of file*/