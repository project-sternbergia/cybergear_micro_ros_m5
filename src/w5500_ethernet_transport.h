#ifndef W5500_ETHERNET_TRANSPORT_H
#define W5500_ETHERNET_TRANSPORT_H

#ifdef CONFIG_IDF_TARGET_ESP32

#define CS 19

#include <Arduino.h>
#include <EthernetUdp.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <rcl/allocator.h>
#include <micro_ros_arduino.h>
#include "IPAddress.h"

extern "C" bool arduino_w5500_ethernet_udp_transport_open(struct uxrCustomTransport * transport);
extern "C" bool arduino_w5500_ethernet_udp_transport_close(struct uxrCustomTransport * transport);
extern "C" size_t arduino_w5500_ethernet_udp_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern "C" size_t arduino_w5500_ethernet_udp_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

static inline void set_microros_w5500_ethernet_udp_transports(byte mac[], IPAddress client_ip, IPAddress agent_ip, uint16_t agent_port)
{
	static struct micro_ros_agent_locator locator;
  Ethernet.init(CS);
  Ethernet.begin(mac, client_ip);
	while (Ethernet.linkStatus() == LinkOFF){
		delay(100);
	}

	locator.address = agent_ip;
	locator.port = agent_port;

	rmw_uros_set_custom_transport(
		false,
		(void *) &locator,
		arduino_w5500_ethernet_udp_transport_open,
		arduino_w5500_ethernet_udp_transport_close,
		arduino_w5500_ethernet_udp_transport_write,
		arduino_w5500_ethernet_udp_transport_read
	);
}

#endif // CONFIG_IDF_TARGET_ESP32
#endif // W5500_ETHERNET_TRANSPORT_H
