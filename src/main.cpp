#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else
#include "USB.h"
#include "Arduino.h"
// #include <pb_arduino.h>
#include "test.pb.h"
#include "pb_common.h"
#include "pb_encode.h"
#include "pb_decode.h"
#if !ARDUINO_USB_CDC_ON_BOOT
#endif
USBCDC USBSerial;

int status = 0;

uint8_t buffer[256];
protoblog_TestMessageWithoutOptions msg = protoblog_TestMessageWithoutOptions_init_zero;
pb_ostream_t ostream;
pb_istream_t istream;
size_t written;

void respond_init_serial(){
  // Kirim data pake crc:
  // Datanya disatukan dulu jadi array buffer
  // Datanya diitung dulu pake CRC
  // Tambahin remainder CRC ke datanya
  // Kirim deh
}

static void usbEventCallback(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == ARDUINO_USB_EVENTS) {
    arduino_usb_event_data_t *data = (arduino_usb_event_data_t *)event_data;
    switch (event_id) {
      case ARDUINO_USB_STARTED_EVENT: Serial.println("USB PLUGGED"); break;
      case ARDUINO_USB_STOPPED_EVENT: Serial.println("USB UNPLUGGED"); break;
      case ARDUINO_USB_SUSPEND_EVENT: Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en); break;
      case ARDUINO_USB_RESUME_EVENT:  Serial.println("USB RESUMED"); break;

      default: break;
    }
  } else if (event_base == ARDUINO_USB_CDC_EVENTS) {
    arduino_usb_cdc_event_data_t *data = (arduino_usb_cdc_event_data_t *)event_data;
    switch (event_id) {
      case ARDUINO_USB_CDC_CONNECTED_EVENT:    
        delay(100);
        msg.address = 1;
        msg.number = 2;
        msg.whatf = 3;
        ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        pb_encode(&ostream, &protoblog_TestMessageWithoutOptions_msg, &msg);
        Serial.write(buffer, ostream.bytes_written);
        Serial.write('/');
        break;
      case ARDUINO_USB_CDC_DISCONNECTED_EVENT: Serial.println("CDC DISCONNECTED"); break;
      case ARDUINO_USB_CDC_LINE_STATE_EVENT:   Serial.printf("CDC LINE STATE: dtr: %u, rts: %u\n", data->line_state.dtr, data->line_state.rts); break;
      case ARDUINO_USB_CDC_LINE_CODING_EVENT:
        Serial.printf(
          "CDC LINE CODING: bit_rate: %lu, data_bits: %u, stop_bits: %u, parity: %u\n", data->line_coding.bit_rate, data->line_coding.data_bits,
          data->line_coding.stop_bits, data->line_coding.parity
        );
        break;
      case ARDUINO_USB_CDC_RX_EVENT:
        
        // if(data->rx == 0x303a) Serial.write(0x303b);
        // Calculate CRC dulu
        // Kalau remainder nol, berarti datanya bener
        // Kalau gak nol, berarti gak bener
        // Pisahin bit CRC nya sama datanya
        // Ubah datanya jadi data yang mau dipake
        {
          uint8_t buf[data->rx.len];
          auto len = USBSerial.read(buffer, data->rx.len);
          Serial.write(buffer, len);
        }
        // // auto size = Serial.readBytesUntil('/n', buffer, ostream.bytes_written);

        // istream = pb_istream_from_buffer(buffer, data->rx.len);

        // TestMessageWithoutOptions decoded = TestMessageWithoutOptions_init_zero;

        // pb_decode(&istream, &TestMessageWithoutOptions_msg, &decoded);

        // // Send a response which increments the decoded buffer
        // delay(10);
        // TestMessageWithoutOptions origin = TestMessageWithoutOptions_init_zero;
        // origin.number = decoded.number + 1;
        // ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        // pb_encode(&ostream, &TestMessageWithoutOptions_msg, &origin);
        // written = ostream.bytes_written;

        // Serial.write(buffer, written);
        break;
      case ARDUINO_USB_CDC_RX_OVERFLOW_EVENT: 
        // if(data->rx_overflow.dropped_bytes == 'a')respond_init_serial();
        // Serial.printf("%x",data->rx_overflow.dropped_bytes);
        Serial.printf("CDC RX Overflow of %d bytes", data->rx_overflow.dropped_bytes); 
        break;

      default: break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  USB.onEvent(usbEventCallback);
  USBSerial.onEvent(usbEventCallback);

  USBSerial.begin();
  USB.begin();
}

void loop() {
  // while (Serial.available()) {
  //   size_t l = Serial.available();
  //   uint8_t b[l];
  //   l = Serial.read(b, l);
  //   USBSerial.write(b, l);
  // }
  // ESP.deepSleep(10);
  // delay(10);
}
#endif /* ARDUINO_USB_MODE */