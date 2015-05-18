
/*----------------------------------INCLUDES----------------------------------*/
#include "math.h"
#include "TinyGPS.h"

/*-------------------ENUM, TYPE AND CONSTANT DEFINITIONS----------------------*/
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000) // 24 hours in millis


/*---------------------------VARIABLE DEFINITIONS-----------------------------*/
int wifiRSSI        = 0;                // wifi signal strength
int lastWifiCheck   = millis() - 5000;  // 5 seconds ago
int interval        = 30000;            // 30 seconds
char* wifiSSID      = "NETWORK_NAME";   // wifi network name

TinyGPS gps;

/*---------------------------FUNCTION DECLARATION-----------------------------*/


/******************************************************************************/
    /**
        @name     setup
        @brief    setup code to only run once
        @return   void

    **/
/******************************************************************************/
void setup(){
    Spark.variable("WiFi_RSSI", &wifiRSSI, INT);
    Spark.variable("WiFi_SSID", wifiSSID, STRING);
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

    if ((now - lastWifiCheck) > interval){
        wifiRSSI = WiFi.RSSI();
        wifiSSID = WiFi.SSID();
        Spark.publish("WiFi_RSSI", String(wifiRSSI));
        Spark.publish("WiFi_SSID", wifiSSID);
        lastWifiCheck = millis();
    }
}
