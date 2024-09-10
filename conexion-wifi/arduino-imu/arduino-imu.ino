#include "WiFi.h"
#include <WiFiClient.h>

const char* ssid = "TurtleBot1";
const char* password = "TurtleBot1";

WiFiServer server(80);

String imuOrientationZ;

void setup() {
  Serial.begin(115200);
  Serial.println();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  IPAddress myIP = WiFi.localIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  WiFiClient client = server.available();  

  if (client) {                             
    Serial.println("New Client.");
    while (client.connected()) {
      imuOrientationZ = client.readStringUntil('\n');
      imuOrientationZ.trim();

      Serial.printf("Orientacion en z de la imu: %s\n", imuOrientationZ);

    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
