int wifiRSSI = 0;   // wifi signal strength
String wifiSSID = "";
int lastWifiCheck = millis() - 5000;    // 5 seconds ago
int interval = 30000;                   // 30 seconds


void setup(){
    Spark.variable("WiFi_RSSI", &wifiRSSI, INT);
    //Spark.variable("WiFi_SSID", wifiSSID, STRING);
}


void loop(){
    int now = millis();

    if ((now - lastWifiCheck) > interval){
        wifiRSSI = WiFi.RSSI();
        wifiSSID = WiFi.SSID();
        Spark.publish("WiFi_RSSI", String(wifiRSSI));
        lastWifiCheck = millis();
    }
}
