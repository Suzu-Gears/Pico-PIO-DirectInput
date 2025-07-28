/*
https://github.com/sekigon-gonnoc/Pico-PIO-USB/blob/main/examples/arduino/device_info/device_info.ino

必須ライブラリ: Arduino IDEでインストール
  Adafruit TinyUSB Library
  Pico PIO USB

Arduino IDE設定:
  ツール - USB Stack: Adafruit TinyUSB
  ツール - CPU Speed: 120MHzの倍数(120MHzでいいと思う)

  F310は6分から7分の間で接続が切れるっぽい(原因がマイコン側かF310側かは不明)
  5分でリセットを繰り返すと30分以上放置してからコントローラ触っても使えた
*/

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "tusb.h"

#include <Adafruit_NeoPixel.h>
#define DIN_PIN 16   // NeoPixel　の出力ピン番号はGP23
#define LED_COUNT 1  // LEDの連結数
Adafruit_NeoPixel pixels(LED_COUNT, DIN_PIN, NEO_GRB + NEO_KHZ800);

#define HOST_PIN_DP 12  // DMはDPのGP番号+1のピンになる この場合はGP13

const unsigned long RESET_TIME = 5 * 60 * 1000;  //(5分)  上限は4294967295ミリ秒 約49日

Adafruit_USBH_Host USBHost;

volatile uint32_t last_input_time = 0;
volatile uint32_t last_LED_blink_time = 0;
volatile bool usb_reconnecting = false;

void init_usb_host() {
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  pio_cfg.sm_tx = 3;
  pio_cfg.sm_rx = 2;
  pio_cfg.sm_eop = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch = 9;
#endif
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);
}

// HIDデバイスがマウントされたとき呼ばれる
extern "C" void tuh_hid_mount_cb(uint8_t daddr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  tuh_hid_parse_report_descriptor(NULL, 0, desc_report, desc_len);
  tuh_hid_receive_report(daddr, instance);
}

// HIDレポート受信時に呼ばれる
extern "C" void tuh_hid_report_received_cb(uint8_t daddr, uint8_t instance, uint8_t const *report, uint16_t len) {
  last_input_time = millis();
  pixels.setPixelColor(0, pixels.Color(32, 0, 0));
  pixels.show();
  for (uint16_t i = 0; i < len; i++) {
    Serial1.printf("%02X ", report[i]);
  }
  Serial1.println();
  tuh_hid_receive_report(daddr, instance);
}

void setup() {
  Serial1.begin(115200);
  while (!Serial1) delay(10);
  last_input_time = millis();

  pixels.begin();  //NeoPixel制御開始
}

void loop() {
  // Serial1.println(millis() - last_input_time);

  if (millis() - last_LED_blink_time > 1000) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
  }
  if (millis() - last_LED_blink_time > 2000) {
    pixels.setPixelColor(0, pixels.Color(0, 32, 0));
    pixels.show();
    last_LED_blink_time = millis();
  }

  if (!usb_reconnecting && (millis() - last_input_time > RESET_TIME)) {
    usb_reconnecting = true;
    Serial1.println("System reset...");
    watchdog_reboot(0, 0, 10);  // https://okiraku-camera.tokyo/blog/?p=15096
    while (1);                  // リセット待ち
    usb_reconnecting = false;
  }
}

void setup1() {
  while (!Serial1) delay(10);
  init_usb_host();
}

void loop1() {
  USBHost.task();
}
