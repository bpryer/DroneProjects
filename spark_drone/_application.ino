
/*----------------------------------INCLUDES----------------------------------*/
#include "math.h"
#include "TinyGPS.h"
#include "LSM9DS0.h"
#include "Adafruit_Sensor.h"
/*#include "Adafruit_Sensor.h"
#include "Adafruit_LSM303_U.h"
#include "Adafruit_L3GD20_U.h"
#include "Adafruit_9DOF.h"*/

/*-------------------ENUM, TYPE AND CONSTANT DEFINITIONS----------------------*/
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000) // 24 hours in millis


/*---------------------------VARIABLE DEFINITIONS-----------------------------*/
/* wifi variables */
int     wifiRSSI        = 0;                // wifi signal strength
int     lastWifiCheck   = millis() - 5000;  // 5 seconds ago
int     interval        = 30000;            // 30 seconds
char*   wifiSSID        = "NETWORK_NAME";   // wifi network name

/* gps variables */
char    szInfo[64];
int     lastGPSCheck    = millis() - 10000; // 10 seconds ago
int     gpsInterval     = 60000;            // Every minute
TinyGPS gps;

/* 9_dof variables */
int lastDofCheck   = millis() - 2000;  // 2 seconds ago
int dofInterval    = 2000;             // 1 second
bool sensorError   = false;            // used to detect sensor errors

/* Assign a unique ID to the sensors */
/*Adafruit_9DOF                   dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified   accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified     mag   = Adafruit_LSM303_Mag_Unified(30302);*/
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/*---------------------------FUNCTION DECLARATION-----------------------------*/
void displaySensorDetails(void);
void configureSensor(void);

/******************************************************************************/
    /**
        @name     setup
        @brief    setup code to only run once
        @return   void

    **/
/******************************************************************************/
void setup(){
    Serial1.begin(9600);    // Serial for GPS communication TX / RX

    /* ---------- Spark Variables ---------- */
    Spark.variable("WiFi_RSSI", &wifiRSSI, INT);
    Spark.variable("WiFi_SSID", wifiSSID, STRING);

    if(!lsm.begin()){
        Spark.publish("9_DOF error.  Check Wiring.");
        sensorError = true;
    }

    displaySensorDetails();
    configureSensor();
}


/******************************************************************************/
    /**
        @name     loop
        @brief    main program code to loop continuously
        @return   void

    **/
/******************************************************************************/
void loop(){
    int now = millis();
    bool isValidGPS = false;

    // check if we need to update 9_dof
    if(((now - lastDofCheck) > dofInterval) && !sensorError){
        /* Get a new sensor event */
        sensors_event_t accel, mag, gyro, temp;

        lsm.getEvent(&accel, &mag, &gyro, &temp);

        Spark.publish("temp", String(temp.temperature));

        lastDofCheck = millis();
    }

    // check if we need to update gps reading
    if((now - lastGPSCheck) > gpsInterval){

        for (unsigned long start = millis(); millis() - start < 1000;){
            // Check GPS data is available
            while (Serial1.available()){
                char c = Serial1.read();

                // parse GPS data
                if (gps.encode(c))
                    isValidGPS = true;
            }
        }

        // If we have a valid GPS location then publish it
        if (isValidGPS){
            float lat, lon;
            unsigned long age;

            gps.f_get_position(&lat, &lon, &age);

            sprintf(szInfo, "%.6f,%.6f",
                (lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat),
                (lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon));
        }
        else{
            /*
                Not a valid GPS location, just pass 0.0,0.0
                This is not correct because 0.0,0.0 is a valid GPS location,
                we have to pass a invalid GPS location
                and check it at the client side
            */
            sprintf(szInfo, "0.0,0.0");
        }

        Spark.publish("gpsloc", szInfo);
        lastGPSCheck = millis();
    }

    // check if we need to update wifi signal
    if ((now - lastWifiCheck) > interval){
        wifiRSSI = WiFi.RSSI();
        wifiSSID = WiFi.SSID();
        Spark.publish("WiFi_RSSI", String(wifiRSSI));
        Spark.publish("WiFi_SSID", wifiSSID);
        lastWifiCheck = millis();
    }
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;

  lsm.getSensor(&accel, &mag, &gyro, &temp);

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
