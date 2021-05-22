/*
 * M5S_LCD.cpp
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

#include "../system/SoC.h"

#if defined(M5StackCore2)

#include "M5S_LCD.h"

#include "RF.h"
#include "LED.h"
#include "GNSS.h"
#include "Baro.h"
#include "Battery.h"
#include "../TrafficHelper.h"
#include <M5Core2.h>

enum
{
  M5S_LCD_PAGE_RADIO,
  M5S_LCD_PAGE_OTHER,
#if !defined(EXCLUDE_M5S_LCD_BARO_PAGE)
  M5S_LCD_PAGE_BARO,
#endif /* EXCLUDE_M5S_LCD_BARO_PAGE */
  M5S_LCD_PAGE_COUNT
};

static bool     M5S_LCD_display_titles = false;
static uint32_t prev_tx_packets_counter = (uint32_t) -1;
static uint32_t prev_rx_packets_counter = (uint32_t) -1;
extern uint32_t tx_packets_counter, rx_packets_counter;

static uint32_t prev_acrfts_counter = (uint32_t) -1;
static uint32_t prev_sats_counter   = (uint32_t) -1;
static uint32_t prev_uptime_minutes = (uint32_t) -1;
static int32_t  prev_voltage        = (uint32_t) -1;
static int8_t   prev_fix            = (uint8_t)  -1;

#if !defined(EXCLUDE_M5S_LCD_BARO_PAGE)
static int32_t  prev_altitude       = (int32_t)   -10000;
static int32_t  prev_temperature    = (int32_t)   -100;
static uint32_t prev_pressure       = (uint32_t)  -1;
static int32_t  prev_cdr            = (int32_t)   -10000; /* climb/descent rate */
#endif /* EXCLUDE_M5S_LCD_BARO_PAGE */

unsigned long M5S_LCDTimeMarker = 0;

const char *M5S_LCD_Protocol_ID[] = {
  [RF_PROTOCOL_LEGACY]    = "L",
  [RF_PROTOCOL_OGNTP]     = "O",
  [RF_PROTOCOL_P3I]       = "P",
  [RF_PROTOCOL_ADSB_1090] = "A",
  [RF_PROTOCOL_ADSB_UAT]  = "U",
  [RF_PROTOCOL_FANET]     = "F"
};

const char *ISO3166_CC[] = {
  [RF_BAND_AUTO] = "--",
  [RF_BAND_EU]   = "EU",
  [RF_BAND_US]   = "US",
  [RF_BAND_AU]   = "AU",
  [RF_BAND_NZ]   = "NZ",
  [RF_BAND_RU]   = "RU",
  [RF_BAND_CN]   = "CN",
  [RF_BAND_UK]   = "UK",
  [RF_BAND_IN]   = "IN",
  [RF_BAND_IL]   = "IL",
  [RF_BAND_KR]   = "KR"
};

const char SoftRF_text[]   = "SoftRF";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";
const char ACFTS_text[]    = "ACFTS";
const char SATS_text[]     = "SATS";
const char FIX_text[]      = "FIX";
const char UPTIME_text[]   = "UPTIME";
const char BAT_text[]      = "BAT";

#if !defined(EXCLUDE_M5S_LCD_BARO_PAGE)
const char ALT_text[]      = "ALT M";
const char TEMP_text[]     = "TEMP C";
const char PRES_text[]     = "PRES MB";
const char CDR_text[]      = "CDR FPM";
#endif /* EXCLUDE_M5S_LCD_BARO_PAGE */

static const uint8_t Dot_Tile[] = { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 };

static int M5S_LCD_current_page = M5S_LCD_PAGE_RADIO;
//static int M5S_LCD_current_page = M5S_LCD_PAGE_BARO;
//static int M5S_LCD_current_page = M5S_LCD_PAGE_OTHER;

static void M5S_LCD_radio()
{
  char buf[16];
  uint32_t disp_value;

  switch (hw_info.display)
  {
  case DISPLAY_M5S_C2:
  default:
    if (!M5S_LCD_display_titles) {

      M5.Lcd.fillScreen(WHITE);

      M5.Lcd.drawString(ID_text,10,10,1);
      M5.Lcd.drawString(PROTOCOL_text,120,10,1);
      M5.Lcd.drawString(RX_text,10,64,1);
      M5.Lcd.drawString(TX_text,120,64,1);

      snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr & 0x00FFFFFF);
      M5.Lcd.drawString(buf,10,26,2);

      M5.Lcd.drawString(M5S_LCD_Protocol_ID[ThisAircraft.protocol],120,26,2);


      if (settings->power_save & POWER_SAVE_NORECEIVE &&
          (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
        M5.Lcd.drawString("OFF",10,80,2);
        prev_rx_packets_counter = rx_packets_counter;
      } else {
        prev_rx_packets_counter = (uint32_t) -1;
      }

      if (settings->mode        == SOFTRF_MODE_RECEIVER ||
          settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
          settings->txpower     == RF_TX_POWER_OFF) {
        M5.Lcd.drawString("OFF",120,80,2);
        prev_tx_packets_counter = tx_packets_counter;
      } else {
        prev_tx_packets_counter = (uint32_t) -1;
      }

      M5S_LCD_display_titles = true;
    }

    if (rx_packets_counter != prev_rx_packets_counter) {
      disp_value = rx_packets_counter % 1000;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR("  "));
      } else {
        if (disp_value < 100) {
          strcat_P(buf,PSTR(" "));
        };
      }

      M5.Lcd.drawString(buf,10,80,2);
      prev_rx_packets_counter = rx_packets_counter;
    }
    if (tx_packets_counter != prev_tx_packets_counter) {
      disp_value = tx_packets_counter % 1000;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR("  "));
      } else {
        if (disp_value < 100) {
          strcat_P(buf,PSTR(" "));
        };
      }

      M5.Lcd.drawString(buf,120,80,2);
      prev_tx_packets_counter = tx_packets_counter;
    }

    break;
  }
}

static void M5S_LCD_other()
{
  char buf[16];
  uint32_t disp_value;

  switch (hw_info.display)
  {
  case DISPLAY_M5S_C2:
  default:
    if (!M5S_LCD_display_titles) {

      M5.Lcd.fillScreen(WHITE);

      M5.Lcd.drawString(ACFTS_text,10,10,1);
      M5.Lcd.drawString(SATS_text,80,10,1);
      M5.Lcd.drawString(FIX_text,140,10,1);
      M5.Lcd.drawString(UPTIME_text,10,64,1);
      M5.Lcd.drawString(BAT_text,140,64,1);

      prev_acrfts_counter = (uint32_t) -1;
      prev_sats_counter   = (uint32_t) -1;
      prev_fix            = (uint8_t)  -1;
      prev_uptime_minutes = (uint32_t) -1;
      prev_voltage        = (uint32_t) -1;

      M5S_LCD_display_titles = true;
    }

    uint32_t acrfts_counter = Traffic_Count();
    uint32_t sats_counter   = gnss.satellites.value();
    uint8_t  fix            = (uint8_t) isValidGNSSFix();
    uint32_t uptime_minutes = millis() / 60000;
    int32_t  voltage        = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
// causes crash ???                                (int) ((Battery_voltage()+0.05) * 10.0) : 0;
                                (int) (Battery_voltage() * 10.0) : 0;

    if (prev_acrfts_counter != acrfts_counter) {
      disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      }
      M5.Lcd.drawString(buf,10,26,2);
      prev_acrfts_counter = acrfts_counter;
    }

    if (prev_sats_counter != sats_counter) {
      disp_value = sats_counter > 99 ? 99 : sats_counter;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      }
      M5.Lcd.drawString(buf,80,26,2);
      prev_sats_counter = sats_counter;
    }

    if (prev_fix != fix) {
      M5.Lcd.drawChar(fix > 0 ? '+' : '-',140,26,2);
      prev_fix = fix;
    }

    if (prev_uptime_minutes != uptime_minutes) {
      uint32_t uptime_hours = uptime_minutes / 60;
      disp_value = uptime_hours % 100;
      if (disp_value < 10) {
        buf[0] = '0';
        itoa(disp_value, buf+1, 10);
      } else {
        itoa(disp_value, buf, 10);
      }
      buf[2] = ':';
      disp_value = uptime_minutes % 60;
      if (disp_value < 10) {
        buf[3] = '0';
        itoa(disp_value, buf+4, 10);
      } else {
        itoa(disp_value, buf+3, 10);
      }
      M5.Lcd.drawString(buf,10,80,2);
      prev_uptime_minutes = uptime_minutes;
    }

    if (prev_voltage != voltage) {
      if (voltage) {
        disp_value = voltage / 10;
        disp_value = disp_value > 9 ? 9 : disp_value;
        itoa(disp_value, buf, 10);
        buf[1] = '.';
        disp_value = voltage % 10;
        itoa(disp_value, buf+2, 10);
        M5.Lcd.drawString(buf,140,80,2);
      } else {
        M5.Lcd.drawString("N/A",140,80,2);
      }
      prev_voltage = voltage;
    }

    break;
  }
}

#if !defined(EXCLUDE_M5S_LCD_BARO_PAGE)
static void M5S_LCD_baro()
{
  char buf[16];

  switch (hw_info.display)
  {
  case DISPLAY_M5S_C2:
  default:
    if (!M5S_LCD_display_titles) {

      M5.Lcd.fillScreen(WHITE);

      M5.Lcd.drawString(ALT_text,10,10,1);
      M5.Lcd.drawString(TEMP_text,120,10,1);
      M5.Lcd.drawString(PRES_text,10,64,1);
      M5.Lcd.drawString(CDR_text,120,64,1);

      prev_altitude     = (int32_t)   -10000;
      prev_temperature  = prev_altitude;
      prev_pressure     = (uint32_t)  -1;
      prev_cdr          = prev_altitude;

      M5S_LCD_display_titles = true;
    }

    int32_t altitude    = Baro_altitude();        /* metres */
    int32_t temperature = Baro_temperature();     /* Celcius */
    uint32_t pressure   = Baro_pressure() / 100;  /* mbar */
    int32_t cdr         = ThisAircraft.vs;        /* feet per minute */

    if (prev_altitude != altitude) {
      snprintf(buf, sizeof(buf), "%4d", altitude);
      M5.Lcd.drawString(buf,10,26,2);
      prev_altitude = altitude;
    }

    if (prev_temperature != temperature) {
      snprintf(buf, sizeof(buf), "%3d", temperature);
      M5.Lcd.drawString(buf,120,26,2);
      prev_temperature = temperature;
    }

    if (prev_pressure != pressure) {
      snprintf(buf, sizeof(buf), "%4d", pressure);
      M5.Lcd.drawString(buf,10,80,2);
      prev_pressure = pressure;
    }

    if (prev_cdr != cdr) {
      int disp_value = constrain(cdr, -999, 999);
      snprintf(buf, sizeof(buf), "%4d", disp_value);
      M5.Lcd.drawString(buf,120,80,2);
      prev_cdr = cdr;
    }

    break;
  }
}
#endif /* EXCLUDE_M5S_LCD_BARO_PAGE */

void M5S_LCD_loop()
{
    if (isTimeToM5S_LCD()) {
      switch (M5S_LCD_current_page)
      {
      case M5S_LCD_PAGE_OTHER:
        M5S_LCD_other();
        break;
#if !defined(EXCLUDE_M5S_LCD_BARO_PAGE)
      case M5S_LCD_PAGE_BARO:
        M5S_LCD_baro();
        break;
#endif /* EXCLUDE_M5S_LCD_BARO_PAGE */
      case M5S_LCD_PAGE_RADIO:
      default:
        M5S_LCD_radio();
        break;
      }

      M5S_LCDTimeMarker = millis();
    }
}

void M5S_LCD_fini(int reason)
{
    switch (hw_info.display)
    {
    case DISPLAY_M5S_C2:
    default:
      M5.Lcd.fillScreen(WHITE);
      M5.Lcd.setCursor(10, 10);
      M5.Lcd.print(reason == SOFTRF_SHUTDOWN_LOWBAT ?
                                "LOW BAT" : "  OFF  ");
      break;
    }
}

void M5S_LCD_info1()
{
    switch (hw_info.display)
    {
    case DISPLAY_M5S_C2:
    default:

      M5.Lcd.fillScreen(WHITE);
      M5.Lcd.setCursor(10, 10);
      M5.Lcd.print("RADIO");
      M5.Lcd.setCursor(80, 10);
      M5.Lcd.print(hw_info.rf   != RF_IC_NONE       ? "+" : "-");
      M5.Lcd.setCursor(10, 26);
      M5.Lcd.print("GNSS");
      M5.Lcd.setCursor(80, 26);
      M5.Lcd.print(hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
      M5.Lcd.setCursor(10, 42);
      M5.Lcd.print("LCD");
      M5.Lcd.setCursor(80, 42);
      M5.Lcd.print(hw_info.display != DISPLAY_NONE  ? "+" : "-");
      M5.Lcd.setCursor(10, 58);
      M5.Lcd.print("BARO");
      M5.Lcd.setCursor(80, 58);
      M5.Lcd.print(hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

      break;
    }

    delay(3000);
}

void M5S_LCD_Next_Page()
{
    M5S_LCD_current_page = (M5S_LCD_current_page + 1) % M5S_LCD_PAGE_COUNT;
#if !defined(EXCLUDE_M5S_LCD_BARO_PAGE)
    if (M5S_LCD_current_page == M5S_LCD_PAGE_BARO && hw_info.baro == BARO_MODULE_NONE) {
      M5S_LCD_current_page = (M5S_LCD_current_page + 1) % M5S_LCD_PAGE_COUNT;
    }
#endif /* EXCLUDE_M5S_LCD_BARO_PAGE */
    M5S_LCD_display_titles = false;
}

#endif /* USE_M5S_LCD */
