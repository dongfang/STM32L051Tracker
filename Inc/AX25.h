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

#ifndef __AX25_H__
#define __AX25_H__

#include "Types.h"
#include <stdint.h>

void ax25_begin(uint16_t txDelay);
void ax25_send_header(const AX25_Address_t *addresses[], int num_addresses);
void ax25_send_byte(uint8_t byte);
void ax25_send_string(const char *string);
void ax25_end();

#define MAX_PACKET (256*3/2)
extern volatile uint8_t packet[MAX_PACKET];
extern volatile uint16_t packet_size;
extern volatile uint16_t packet_cnt;

#endif
