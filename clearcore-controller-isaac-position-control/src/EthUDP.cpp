/*
 * EthUDP.cpp
 *
 * Created: 9/21/2023 6:20:23 PM
 * Author: Luke Strohbehn
 */ 

#ifndef __SERIAL_DEBUG__
#define __SERIAL_DEBUG__ 0
#endif

#include "EthUDP.h"
#include <stdio.h>
#include <stdlib.h>

EthUDP::EthUDP():
	/* Initialize class variables */
	m_local_ip(169, 254, 57, 177),
	m_local_port {8888},
	m_remote_ip(169, 254, 57, 209),
	m_remote_port {44644}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip):
	m_local_ip(_local_ip),
	m_local_port {8888},
	m_remote_ip(169, 254, 57, 209),
	m_remote_port {44644}
{
	
	
}

EthUDP::EthUDP(IpAddress _local_ip, int _local_port):
	m_local_ip(_local_ip),
	m_local_port(_local_port),
	m_remote_ip(169, 254, 57, 209),
	m_remote_port {44644}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip, IpAddress _remote_ip):
	m_local_ip(_local_ip),
	m_local_port {8888},
	m_remote_ip(_remote_ip),
	m_remote_port {44644}
{
	
}

EthUDP::EthUDP(IpAddress _local_ip, int _local_port, IpAddress _remote_ip, int _remote_port):
	m_local_ip(_local_ip),
	m_local_port(_local_port),
	m_remote_ip(_remote_ip),
	m_remote_port(_remote_port)
{

}

EthUDP::~EthUDP() {}

void EthUDP::begin(void) {
	/* Set up UDP Ethernet communication */
	if (m_started) {
		return;
	}

	// Check physical Ethernet link
	if (!EthernetMgr.PhyLinkActive()) {
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
		ConnectorUsb.SendLine("Could not detect a physical Ethernet connection.");
#endif
		return;
	}
	
	// Run the setup for the ClearCore Ethernet manager
	EthernetMgr.Setup();
	if (m_using_dhcp) {
		bool dhcp_success = EthernetMgr.DhcpBegin();
		if (dhcp_success) {
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
			ConnectorUsb.Send("DHCP successfully assigned an IP address: ");
			ConnectorUsb.SendLine(EthernetMgr.LocalIp().StringValue());
#endif
		}
		else {
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
			ConnectorUsb.SendLine("DHCP configuration was unsuccessful.");
#endif
			return;
		}
	}
	else {
		EthernetMgr.LocalIp(m_local_ip);
		//EthernetMgr.GatewayIp(IpAddress(169,254, 93, 234)); // TODO: add these to the constructors
		EthernetMgr.NetmaskIp(IpAddress(255, 255, 0, 0));
	}
	
	// Begin listening on the local port for UDP datagrams
	udp.Begin(m_local_port);
	m_started = true;
}


void EthUDP::read_packet(slidersystem::DataInterface* command_interface) {
	/* Look for a received packet, store in 'received_packet' if present */
	if (!m_started) {
		begin();
		return;
	}

	uint16_t packet_size = udp.PacketParse();
	if (packet_size > 0) {
		if (packet_size >= MAX_PACKET_LENGTH) {
			packet_size = MAX_PACKET_LENGTH - 1;
		}
		udp.PacketRead(m_received_packet, packet_size);
		m_received_packet[packet_size] = '\0';

		// Reply to the host that actually sent the latest command. The constructor
		// default remains useful for boot/status packets before the first command.
		m_remote_ip = udp.RemoteIp();
		m_remote_port = udp.RemotePort();
		m_have_remote_endpoint = true;
				
		// Parse data from the received packet
		// Extract first field
		char* received_packet_cstr = reinterpret_cast<char*>(m_received_packet);
		m_token = strtok(received_packet_cstr, m_delimiter);

		
		if (m_token != NULL) {
			command_interface->system_status = static_cast<slidersystem::SystemStatus>(atoi(m_token));			
			// Extract second field
			m_token = strtok(NULL, m_delimiter);
			//token_cstr = reinterpret_cast<char*>(token);
			if (m_token != NULL) {
				command_interface->pos_steps = static_cast<int32_t>(atol(m_token));
				new_data = true;
			}
		} 		 
	}
}

void EthUDP::construct_data_msg(slidersystem::DataInterface* state) {
	/* Construct the message to send to the ROS2 Node on the host computer 
	https://stackoverflow.com/questions/23966080/sending-struct-over-udp-c
	*/
	// Reset buffers
	memset(&m_msg_buf[0], 0, sizeof(m_msg_buf));
	memset(&m_status_buf[0], 0, sizeof(m_status_buf));
	memset(&m_pos_steps_buf[0], 0, sizeof(m_pos_steps_buf));
	
	// Set data
	sprintf(m_status_buf, "%d", state->system_status);
	sprintf(m_pos_steps_buf, "%ld", static_cast<long>(state->pos_steps));
	
	// Create c-str msg. ROS expects "pos_steps" for position feedback.
	sprintf(m_msg_buf, "{\"status\":%s,\"pos_steps\":%s}", m_status_buf, m_pos_steps_buf);
	
#if __SERIAL_DEBUG__ // || __ETHUDP_DEBUG__
	ConnectorUsb.Send("Constructed msg: ");
	ConnectorUsb.SendLine(m_msg_buf);
#endif
}


void EthUDP::send_packet(slidersystem::DataInterface* state) {
	/* Send a packet */
	if (!m_started) {
		begin();
		return;
	}

	construct_data_msg(state);
#if __SERIAL_DEBUG__ || __ETHUDP_DEBUG__
	ConnectorUsb.Send("Sending msg: ");
	ConnectorUsb.SendLine(m_msg_buf);
#endif
	udp.Connect(m_remote_ip, m_remote_port);
	udp.PacketWrite(m_msg_buf);
	udp.PacketSend();
} 
