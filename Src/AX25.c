/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "AX25.h"

// Module globals
static uint16_t crc __attribute__((section (".noinit")));
static uint8_t ones_in_a_row __attribute__((section (".noinit")));

volatile uint8_t packet[MAX_PACKET] __attribute__((section (".noinit")));
volatile uint16_t packet_size __attribute__((section (".noinit")));
volatile uint16_t packet_cnt __attribute__((section (".noinit")));

// Module functions
static void update_crc(uint8_t a_bit) {
	crc ^= a_bit;
	if (crc & 1)
		crc = (crc >> 1) ^ 0x8408;  // X-modem CRC poly
	else
		crc = crc >> 1;
}

void ax25_send_byte(uint8_t a_byte) {
	uint8_t i = 0;
	while (i++ < 8) {
		uint8_t a_bit = a_byte & 1;
		a_byte >>= 1;
		update_crc(a_bit);
		if (a_bit) {
			// Next bit is a '1'
			if (packet_size >= MAX_PACKET * 8)  // Prevent buffer overrun
				return;
			packet[packet_size >> 3] |= (1 << (packet_size & 7));
			packet_size++;
			if (++ones_in_a_row < 5)
				continue;
		}
		// Next bit is a '0' or a zero padding after 5 ones in a row
		if (packet_size >= MAX_PACKET * 8)    // Prevent buffer overrun
			return;
		packet[packet_size >> 3] &= ~(1 << (packet_size & 7));
		packet_size++;
		ones_in_a_row = 0;
	}
	// trace_printf("char %c, packetsize %d\n", a_byte, packet_size);
}

void ax25_send_flag() {
	uint8_t flag = 0x7e;
	int i;
	for (i = 0; i < 8; i++, packet_size++) {
		if (packet_size >= MAX_PACKET * 8)  // Prevent buffer overrun
			return;
		if ((flag >> i) & 1)
			packet[packet_size >> 3] |= (1 << (packet_size & 7));
		else
			packet[packet_size >> 3] &= ~(1 << (packet_size & 7));
	}
}

void ax25_diddle(int numBytes) {
	int i;
	for (i = 0; i < numBytes; i++) {
		packet[i] = 0;
	}
	packet_size = i * 8;
}

void ax25_send_string(const char *string) {
	int i;
	for (i = 0; string[i]; i++) {
		ax25_send_byte(string[i]);
	}
}

void ax25_begin(uint16_t txDelay) {
	packet_size = 0;
	ones_in_a_row = 0;
	crc = 0xffff;
	// Send flags during TX_DELAY milliseconds (8 bit-flag = 8000/1200 ms)
	ax25_diddle(txDelay);
	ax25_send_flag();
}

void ax25_send_header(const AX25_Address_t *addresses[], int num_addresses) {
	int i, j;

	for (i = 0; i < num_addresses; i++) {
		// Transmit callsign
		for (j = 0; addresses[i]->callsign[j]; j++)
			ax25_send_byte(addresses[i]->callsign[j] << 1);
		// Transmit pad
		for (; j < 6; j++)
			ax25_send_byte(' ' << 1);
		// Transmit SSID. Termination signaled with last bit = 1
		if (i == num_addresses - 1)
			ax25_send_byte(('0' + addresses[i]->ssid) << 1 | 1);
		else
			ax25_send_byte(('0' + addresses[i]->ssid) << 1);
	}

	// Control field: 3 = APRS-UI frame
	ax25_send_byte(0x03);

	// Protocol ID: 0xf0 = no layer 3 data
	ax25_send_byte(0xf0);
}

void ax25_end() {
	// Save the crc so that it can be treated it atomically
	uint16_t final_crc = crc;

	// Send the CRC
	ax25_send_byte(~(final_crc & 0xff));
	final_crc >>= 8;
	ax25_send_byte(~(final_crc & 0xff));

	// Signal the end of frame
	ax25_send_flag();
}

