/*
 * Platform_RPi.h
 * Copyright (C) 2018 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(RASPBERRY_PI)

#ifndef PLATFORM_RPI_H
#define PLATFORM_RPI_H

#define swSer Serial

/* Dragino LoRa/GPS HAT */
#if 0 /* WiringPi */
#define SOC_GPIO_PIN_MOSI     12
#define SOC_GPIO_PIN_MISO     13
#define SOC_GPIO_PIN_SCK      14
#define SOC_GPIO_PIN_SS       6
#define SOC_GPIO_PIN_RST      0
#define SOC_GPIO_PIN_DIO0     7
#else /* BCM */
#define SOC_GPIO_PIN_MOSI     RPI_V2_GPIO_P1_19
#define SOC_GPIO_PIN_MISO     RPI_V2_GPIO_P1_21
#define SOC_GPIO_PIN_SCK      RPI_V2_GPIO_P1_23
#define SOC_GPIO_PIN_SS       RPI_V2_GPIO_P1_22 // Slave Select on GPIO25 so P1 connector pin #22
#define SOC_GPIO_PIN_RST      RPI_V2_GPIO_P1_11 // Reset on GPIO17 so P1 connector pin #11
#define SOC_GPIO_PIN_DIO0     RPI_V2_GPIO_P1_7  // IRQ on GPIO4 so P1 connector pin #7
#endif /* GPIO */

#define SOC_GPIO_PIN_MODE_PULLDOWN INPUT
#define SOC_GPIO_PIN_GNSS_PPS SOC_UNUSED_PIN

#endif /* PLATFORM_RPI_H */

#endif /* RASPBERRY_PI */
