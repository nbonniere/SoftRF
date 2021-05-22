/*
 * M5S_LCD.h
 * Copyright (C) 2019-2021 Linar Yusupov
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

#ifndef M5S_LCD_H
#define M5S_LCD_H

#define isTimeToM5S_LCD()          (millis() - M5S_LCDTimeMarker > 500)

byte M5S_LCD_setup(void);
void M5S_LCD_loop(void);
void M5S_LCD_fini(int);
void M5S_LCD_info1(void);
void M5S_LCD_Next_Page(void);

extern const char *M5S_LCD_Protocol_ID[];
extern const char *ISO3166_CC[];
extern const char SoftRF_text[];
extern const char ID_text[];
extern const char PROTOCOL_text[];
extern const char RX_text[];
extern const char TX_text[];

#endif /* M5S_LCD */
