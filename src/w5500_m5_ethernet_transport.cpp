#include "w5500_m5_ethernet_transport.h"
#ifdef CONFIG_IDF_TARGET_ESP32S3
#include <Arduino.h>
#include <M5_Ethernet.h>
#include <micro_ros_arduino.h>

extern "C" {
static EthernetUDP udp_client;

bool arduino_w5500_m5_ethernet_udp_transport_open(struct uxrCustomTransport * transport)
{
  struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *)transport->args;
  udp_client.begin(locator->port);
  return true;
}

bool arduino_w5500_m5_ethernet_udp_transport_close(struct uxrCustomTransport * transport)
{
  udp_client.stop();
  return true;
}

size_t arduino_w5500_m5_ethernet_udp_transport_write(
  struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * errcode)
{
  (void)errcode;
  struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *)transport->args;

  udp_client.beginPacket(locator->address, locator->port);
  size_t sent = udp_client.write(buf, len);
  udp_client.endPacket();
  udp_client.flush();
  return sent;
}

size_t arduino_w5500_m5_ethernet_udp_transport_read(
  struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * errcode)
{
  (void)errcode;
  uint32_t start_time = millis();
  while (millis() - start_time < timeout && udp_client.parsePacket() == 0) {
    delay(1);
  }

  int readed = udp_client.read(buf, len);
  return (readed < 0) ? 0 : readed;
}
}

#endif  // CONFIG_IDF_TARGET_ESP32S3
