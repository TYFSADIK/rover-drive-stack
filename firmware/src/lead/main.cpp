// Lead ESP32 (Heltec V3, SX1262): takes the lead rover's pose from its Pi 5 over UART and broadcasts
// it at 10 Hz on LoRa AND ESP-NOW (same bytes, same seq) so the follower can dedupe.
// If the Pi stops sending pose for 0.5 s the lead stops broadcasting: a stale pose is worse than none.
#include <Arduino.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <esp_now.h>
#include "uart_frame.h"

using namespace convoy;

SX1262 radio = new Module(8, 14, 12, 13);
constexpr int PI_RX = 47, PI_TX = 48;
HardwareSerial FromPi(1);
FrameParser parser;
uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint16_t seq = 0;
Pose4 pose{};
uint32_t pose_ms = 0, last_tx = 0;

void setup() {
  Serial.begin(115200);
  FromPi.begin(921600, SERIAL_8N1, PI_RX, PI_TX);
  radio.begin(915.0, 125.0, 9, 7, 0x12, 14);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, bcast, 6);
  esp_now_add_peer(&peer);
}

void loop() {
  while (FromPi.available())
    if (parser.push(FromPi.read()) == FrameParser::Kind::Pose) { pose = parser.pose; pose_ms = millis(); }
  if (millis() - last_tx >= 100 && pose_ms && millis() - pose_ms < 500) {
    last_tx = millis();
    StatePacket p; p.seq = ++seq; p.t_ms = millis();
    p.x_m = pose.x; p.y_m = pose.y; p.heading_rad = pose.heading; p.speed_mps = pose.speed;
    seal(p);
    esp_now_send(bcast, reinterpret_cast<uint8_t*>(&p), sizeof(p));
    radio.transmit(reinterpret_cast<uint8_t*>(&p), sizeof(p));   // ~150 ms airtime at SF9: lower SF for 10 Hz if needed
  }
}
