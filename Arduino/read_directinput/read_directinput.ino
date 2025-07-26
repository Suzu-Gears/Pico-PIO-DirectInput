/*
https://github.com/sekigon-gonnoc/Pico-PIO-USB/blob/main/examples/arduino/device_info/device_info.ino

必須ライブラリ:
  Adafruit TinyUSB Library
  Pico PIO USB

Arduino IDE設定:
  ツール - USB Stack: Adafruit TinyUSB
  ツール - CPU Speed: 120MHzの倍数(120MHzでいいと思う)
*/

#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"

#define HOST_PIN_DP 12  // Pin used as D+ for host, D- = D+ + 1

#define LANGUAGE_ID 0x0409  // English

// USB Host object
Adafruit_USBH_Host USBHost;

// holding device descriptor
tusb_desc_device_t desc_device;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial1.begin(115200);

  Serial.begin(115200);
  while (!Serial)
    delay(10);  // wait for native usb

  Serial.println("TinyUSB Dual Device Info Example");
}

void loop() {
}

// core1's setup
void setup1() {
  while (!Serial)
    delay(10);  // wait for native usb
  Serial.println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if (cpu_hz != 120000000UL && cpu_hz != 240000000UL) {
    while (!Serial)
      delay(10);  // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while (1)
      delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46 */
  pio_cfg.sm_tx = 3;
  pio_cfg.sm_rx = 2;
  pio_cfg.sm_eop = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch = 9;
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1() {
  USBHost.task();
}

//--------------------------------------------------------------------+
// Gamepad Raw Report Dump
//--------------------------------------------------------------------+

#include "tusb.h"

uint8_t hid_report_buf[64];

#define MAX_REPORT 4
tuh_hid_report_info_t hid_report_info[MAX_REPORT];
uint8_t hid_report_count = 0;

// HIDデバイスがマウントされたとき呼ばれる
extern "C" void tuh_hid_mount_cb(uint8_t daddr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  Serial.printf("HID device mounted: addr=%d, inst=%d\r\n", daddr, instance);

  // レポートディスクリプタを解析
  hid_report_count = tuh_hid_parse_report_descriptor(hid_report_info, MAX_REPORT, desc_report, desc_len);

  // 全てのレポートIDに対して受信要求を出す
  for (uint8_t i = 0; i < hid_report_count; i++) {
    tuh_hid_receive_report(daddr, instance);
  }
}

// HIDレポート受信時に呼ばれる
extern "C" void tuh_hid_report_received_cb(uint8_t daddr, uint8_t instance, uint8_t const *report, uint16_t len) {
  // デバッグ: レポート長と先頭バイトを表示
  // Serial.printf("[DEBUG] len=%u report[0]=%02X\r\n", len, report[0]);

  // すべてのレポートバイトを表示
  for (uint16_t i = 0; i < len; i++) {
    for (uint16_t i = 0; i < len; i++) {
      for (int8_t b = 7; b >= 0; b--) {
        Serial.print((report[i] >> b) & 1);
      }
      Serial.print(" ");
    }
    Serial.println();
  }
  // 次のレポート受信を再度要求
  tuh_hid_receive_report(daddr, instance);
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted (configured)
void tuh_mount_cb(uint8_t daddr) {
  Serial.printf("Device attached, address = %d\r\n", daddr);

  // Get Device Descriptor
  tuh_descriptor_get_device(daddr, &desc_device, 18, print_device_descriptor, 0);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t daddr) {
  Serial.printf("Device removed, address = %d\r\n", daddr);
}

void print_device_descriptor(tuh_xfer_t *xfer) {
  if (XFER_RESULT_SUCCESS != xfer->result) {
    Serial.printf("Failed to get device descriptor\r\n");
    return;
  }

  uint8_t const daddr = xfer->daddr;

  Serial.printf("Device %u: ID %04x:%04x\r\n", daddr, desc_device.idVendor, desc_device.idProduct);
  Serial.printf("Device Descriptor:\r\n");
  Serial.printf("  bLength             %u\r\n", desc_device.bLength);
  Serial.printf("  bDescriptorType     %u\r\n", desc_device.bDescriptorType);
  Serial.printf("  bcdUSB              %04x\r\n", desc_device.bcdUSB);
  Serial.printf("  bDeviceClass        %u\r\n", desc_device.bDeviceClass);
  Serial.printf("  bDeviceSubClass     %u\r\n", desc_device.bDeviceSubClass);
  Serial.printf("  bDeviceProtocol     %u\r\n", desc_device.bDeviceProtocol);
  Serial.printf("  bMaxPacketSize0     %u\r\n", desc_device.bMaxPacketSize0);
  Serial.printf("  idVendor            0x%04x\r\n", desc_device.idVendor);
  Serial.printf("  idProduct           0x%04x\r\n", desc_device.idProduct);
  Serial.printf("  bcdDevice           %04x\r\n", desc_device.bcdDevice);

  // Get String descriptor using Sync API
  uint16_t temp_buf[128];

  Serial.printf("  iManufacturer       %u     ", desc_device.iManufacturer);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_manufacturer_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf))) {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iProduct            %u     ", desc_device.iProduct);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_product_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf))) {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  iSerialNumber       %u     ", desc_device.iSerialNumber);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_serial_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf))) {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial.printf("\r\n");

  Serial.printf("  bNumConfigurations  %u\r\n", desc_device.bNumConfigurations);
}

//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
  // TODO: Check for runover.
  (void)utf8_len;
  // Get the UTF-16 length out of the data itself.

  for (size_t i = 0; i < utf16_len; i++) {
    uint16_t chr = utf16[i];
    if (chr < 0x80) {
      *utf8++ = chr & 0xff;
    } else if (chr < 0x800) {
      *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    } else {
      // TODO: Verify surrogate.
      *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
  size_t total_bytes = 0;
  for (size_t i = 0; i < len; i++) {
    uint16_t chr = buf[i];
    if (chr < 0x80) {
      total_bytes += 1;
    } else if (chr < 0x800) {
      total_bytes += 2;
    } else {
      total_bytes += 3;
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
  return total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len) {
  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  size_t utf8_len = _count_utf8_bytes(temp_buf + 1, utf16_len);

  _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *)temp_buf, sizeof(uint16_t) * buf_len);
  ((uint8_t *)temp_buf)[utf8_len] = '\0';

  Serial.printf((char *)temp_buf);
}
