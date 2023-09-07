#include <WiFi.h>
#include <AsyncUDP.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 135
// pinouts for display (TTGO T-Display ST7789 OLED)
// These pinout can be found at https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#define TFT_CS     5
#define TFT_RST    23
#define TFT_DC     16
#define TFT_MOSI   19
#define TFT_SCLK   18
#define TFT_BL     4  // Display backlight control pin

const char* ssid = "rtc_gang";
const char* password = "roadtrip#1";
const int localPort = 1234;

const IPAddress multicast_ip(224,10,10,10);
const uint16_t multicast_port = 1234;

AsyncUDP udp;

void setup() {
  Serial.begin(115200);

  // wifi mode station
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  if (udp.listenMulticast(multicast_ip, multicast_port)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
    });

    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
  }
}

void loop() {
  // No specific action required in the loop
}