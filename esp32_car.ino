#include <WiFi.h>

/* ==============================
   WIFI CONFIG
   ============================== */

const char* ssid = "SIM";
const char* password = "ARVIN1234";

/* ==============================
   RSU CONFIG
   ============================== */

const char* serverIP = "192.168.137.1";
const int serverPort = 5050;

/* ==============================
   VEHICLE IDENTITY
   ============================== */

// CHANGE PER VEHICLE
int vehicleID = 2;              // 1 = Truck, 2 = Car
String vehicleType = "Car";

/* ==============================
   GLOBALS
   ============================== */

WiFiClient client;

/* ==============================
   SETUP
   ============================== */

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.disconnect(true);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  while (!client.connect(serverIP, serverPort)) {
    delay(500);
  }
}

/* ==============================
   LOOP (SILENT MODE)
   ============================== */

void loop() {

  // Silent reconnect
  if (!client.connected()) {
    while (!client.connect(serverIP, serverPort)) {
      delay(500);
    }
  }

  // Send identity (silent)
  String data = WiFi.macAddress() + "," + String(vehicleID) + "," + vehicleType;
  client.println(data);

  // 🔔 SHOW ONLY ALERTS
  if (client.available()) {
    String alert = client.readStringUntil('\n');

    Serial.println("================================");
    Serial.println("⚠️  COLLISION ALERT");
    Serial.println(alert);
    Serial.println("================================");
  }

  delay(1000);
}
