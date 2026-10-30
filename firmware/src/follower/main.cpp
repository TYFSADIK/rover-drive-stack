// Follower ESP32 (Heltec WiFi LoRa 32 V3, SX1262): RADIO BRIDGE ONLY.
// Receives lead packets on LoRa 915 MHz and ESP-NOW (Wi-Fi fallback), drops corrupt and duplicate
// packets, and forwards survivors to the Pi 5 over UART. The Pi runs the watchdog + controller and
// drives the BTS7960s. The ESP32 never touches the motors.
#include <Arduino.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <esp_now.h>
#include "uart_frame.h"

using namespace convoy;

SX1262 radio = new Module(8, 14, 12, 13);      // Heltec V3: NSS, DIO1, RST, BUSY
constexpr int PI_RX = 47, PI_TX = 48;           // UART1 to Pi GPIO15 (RXD) / GPIO14 (TXD), common GND
HardwareSerial ToPi(1);
volatile bool lora_flag = false;
uint16_t last_seq = 0;
bool have_seq = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR on_lora() { lora_flag = true; }

void forward(const uint8_t* buf, size_t n, Link link, int8_t rssi) {
  if (n != sizeof(StatePacket)) return;
  StatePacket p; memcpy(&p, buf, sizeof(p));
  if (!valid(p)) return;
  portENTER_CRITICAL(&mux);
  bool fresh = !have_seq || newer(p.seq, last_seq);
  if (fresh) { last_seq = p.seq; have_seq = true; }
  portEXIT_CRITICAL(&mux);
  if (!fresh) return;                            // same packet already arrived on the other link
  uint8_t frame[kRxFrameLen];
  ToPi.write(frame, encode_rx(p, link, rssi, frame));
}

void on_espnow(const esp_now_recv_info_t* info, const uint8_t* d, int n) {
  forward(d, n, Link::WiFi, info->rx_ctrl ? int8_t(info->rx_ctrl->rssi) : 0);
}

void setup() {
  Serial.begin(115200);
  ToPi.begin(921600, SERIAL_8N1, PI_RX, PI_TX);
  int st = radio.begin(915.0, 125.0, 9, 7, 0x12, 14);    // SF9/125 kHz, 14 dBm, sync word 0x12 (private)
  if (st != RADIOLIB_ERR_NONE) Serial.printf("radio init failed %d\n", st);
  radio.setDio1Action(on_lora);
  radio.startReceive();
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(on_espnow);
}

void loop() {
  if (lora_flag) {
    lora_flag = false;
    uint8_t buf[64];
    size_t n = radio.getPacketLength();
    if (n <= sizeof(buf) && radio.readData(buf, n) == RADIOLIB_ERR_NONE) forward(buf, n, Link::LoRa, int8_t(radio.getRSSI()));
    radio.startReceive();
  }
}
