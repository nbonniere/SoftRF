/*
 * TrafficHelper.cpp
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

#include <TimeLib.h>

#include "SoCHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "EEPROMHelper.h"
#include "EPDHelper.h"

#include "SkyView.h"

traffic_t ThisAircraft, Container[MAX_TRACKING_OBJECTS], fo, EmptyFO;
traffic_by_dist_t traffic[MAX_TRACKING_OBJECTS];

static unsigned long UpdateTrafficTimeMarker = 0;
static unsigned long Traffic_Voice_TimeMarker = 0;

// Fast approx magnitude (4% max error)
float fast_magnitude(float rel_x, float rel_y)
{
  /* magnitude ~= alpha * max(|rel_x|, |rel_y|) + beta * min(|rel_x|, |rel_y|) */
  static float alpha = 0.96043387;
  static float beta  = 0.39782473;
  float abs_rel_x = fabs(rel_x);
  float abs_rel_y = fabs(rel_y);
  if (abs_rel_x > abs_rel_y) {
    return alpha * abs_rel_x + beta * abs_rel_y;
  } else {
    return alpha * abs_rel_y + beta * abs_rel_x;
  }
}

// Fast approx angle (4 deg max error)
float fast_atan2(float rel_x, float rel_y)
{
  static float ONEQTR_PI = PI / 4.0;
  static float THRQTR_PI = 3.0 * ONEQTR_PI;
  float r, angle;
  float abs_rel_y = fabs(rel_y) + 1e-10f; // kludge to prevent 0/0 condition
  if ( rel_x < 0.0f ) {
    r = (rel_x + abs_rel_y) / (abs_rel_y - rel_x);
	angle = THRQTR_PI;
  } else {
    r = (rel_x - abs_rel_y) / (rel_x + abs_rel_y);
    angle = ONEQTR_PI;
  }
  angle -= ONEQTR_PI * r;
  if ( rel_y < 0.0f ) {
     return( -angle );     // negate if in quad III or IV
  } else {
    return( angle );
  }	
}

// Fast approx angle (0.5 deg max error)
/*
float fast_atan2(float rel_x, float rel_y)
{
  static float ONEQTR_PI = PI / 4.0;
  static float THRQTR_PI = 3.0 * ONEQTR_PI;
  float r, angle;
  float abs_rel_y = fabs(rel_y) + 1e-10f; // kludge to prevent 0/0 condition
  if ( rel_x < 0.0f ) {
	r = (rel_x + abs_rel_y) / (abs_rel_y - rel_x);
	angle = THRQTR_PI;
  } else {
    r = (rel_x - abs_rel_y) / (rel_x + abs_rel_y);
    angle = ONEQTR_PI;
  }
  angle += (0.1963f * r * r - 0.9817f) * r;
  if ( rel_y < 0.0f ) {
     return( -angle );     // negate if in quad III or IV
  } else {
    return( angle );
  }
}
*/

// Fast sine (1% max error)
float fast_sine(float angle)
{
  static float ALPHA = 0.224;
  static float TWO_DIV_PI_SQ = 4/(PI*PI);
  float temp1;
  float temp2;
  while (angle >= PI) {
    angle -= TWO_PI;
  }
  while (angle < -PI) {
    angle += TWO_PI;
  }
  temp1 = fabs(angle);
  temp1 = (PI-temp1)*angle*TWO_DIV_PI_SQ;
  temp2 = fabs(temp1);
  return temp1-(1-temp2)*temp1*ALPHA;
}

float fast_cosine(float angle)
{
  static float PI_DIV_2 = PI / 2;
  return fast_sine(PI_DIV_2-angle);
}

// 2D rotation
void EPD_2D_Rotate(float &tX, float &tY, float tCos, float tSin)
{
    float tTemp;
	tTemp = tX * tCos + tY * -tSin;
    tY = tX * tSin + tY *  tCos;
    tX = tTemp;
}

void Traffic_Add()
{
    if (fo.RelativeDistance > ALARM_ZONE_NONE) {
      return;
    }

    if ( settings->filter == TRAFFIC_FILTER_OFF  ||
        (settings->filter == TRAFFIC_FILTER_500M &&
                      fo.RelativeVertical > -500 &&
                      fo.RelativeVertical <  500) ) {
      int i;

      // if target already in the list, update the list
      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (Container[i].ID == fo.ID) {
          uint8_t alert_bak = Container[i].alert;
          int16_t track_bak = Container[i].Track;
          int16_t speed_bak = Container[i].GroundSpeed;
          Container[i] = fo;
          Container[i].alert = alert_bak;
		  if (fo.Track == -360) {           // in case unkown, use previous value
            Container[i].Track = track_bak;
            Container[i].GroundSpeed = speed_bak;
		  }
          return;
        }
      }

      // when target not in list, check for higher alarm level
	  // or shorter distance, or time-expired target
      int max_dist_ndx = 0;
      int min_level_ndx = 0;
      float max_distance_sq = Container[max_dist_ndx].RelativeDistance;

      for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
        if (now() - Container[i].timestamp > ENTRY_EXPIRATION_TIME) {
          Container[i] = fo;
          return;
        }

        float distance_sq = Container[i].RelativeDistance;

        if  (distance_sq > max_distance_sq) {
          max_dist_ndx = i;
          max_distance_sq = distance_sq;
        }
        if  (Container[i].AlarmLevel < Container[min_level_ndx].AlarmLevel)  {
          min_level_ndx = i;
        }
      }

      if (fo.AlarmLevel > Container[min_level_ndx].AlarmLevel) {
        Container[min_level_ndx] = fo;
        return;
      }

      if (fo.RelativeDistance <  max_distance_sq &&
          fo.AlarmLevel  >= Container[max_dist_ndx].AlarmLevel) {
        Container[max_dist_ndx] = fo;
        return;
      }
    }
}

void Traffic_Update(traffic_t *fop)
{
  fop->RelativeDistance = nmea.distanceBetween( ThisAircraft.latitude,
                                         ThisAircraft.longitude,
                                         fop->latitude,
                                         fop->longitude);

  fop->RelativeBearing  = nmea.courseTo( ThisAircraft.latitude,
                                  ThisAircraft.longitude,
                                  fop->latitude,
                                  fop->longitude);

  fop->RelativeNorth     = fop->RelativeDistance * cos(radians(fop->RelativeBearing));
  fop->RelativeEast      = fop->RelativeDistance * sin(radians(fop->RelativeBearing));
  fop->RelativeVertical  = fop->altitude - ThisAircraft.altitude;
}

static void Traffic_Voice()
{
  int i=0;
  int j=0;
  int bearing;
  char message[80];

  for (i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) <= VOICE_EXPIRATION_TIME) {

      traffic[j].fop = &Container[i];
      traffic[j].distance = Container[i].RelativeDistance;
      j++;
    }
  }

  if (j == 0) { return; }

  const char *u_dist, *u_alt;
  float voc_dist;
  int   voc_alt;
  const char *where;
  char how_far[32];
  char elev[32];

  qsort(traffic, j, sizeof(traffic_by_dist_t), traffic_cmp_by_distance);

  /*
   * Issue only one voice alert per each aircraft in the traffic table.
   * Closest traffic is the first, outmost one is the last.
   */
  for (i=0; i < j; i++) {
    if ((traffic[i].fop->alert & TRAFFIC_ALERT_VOICE) == 0) {

      bearing = (int) (traffic[i].fop->RelativeBearing);

      /* This bearing is always relative to current ground track */
//    if (settings->orientation == DIRECTION_TRACK_UP) {
          bearing -= ThisAircraft.Track;
//    }

      while (bearing < 0) {
        bearing += 360;
      }

      int oclock = ((bearing + 15) % 360) / 30;

      switch (oclock)
      {
      case 0:
        where = "ahead";
        break;
      case 1:
        where = "1oclock";
        break;
      case 2:
        where = "2oclock";
        break;
      case 3:
        where = "3oclock";
        break;
      case 4:
        where = "4oclock";
        break;
      case 5:
        where = "5oclock";
        break;
      case 6:
        where = "6oclock";
        break;
      case 7:
        where = "7oclock";
        break;
      case 8:
        where = "8oclock";
        break;
      case 9:
        where = "9oclock";
        break;
      case 10:
        where = "10oclock";
        break;
      case 11:
        where = "11oclock";
        break;
      }

      switch (settings->units)
      {
      case UNITS_IMPERIAL:
        u_dist = "nautical miles";
        u_alt  = "feet";
        voc_dist = (traffic[i].distance * _GPS_MILES_PER_METER) /
                    _GPS_MPH_PER_KNOT;
        voc_alt  = abs((int) (traffic[i].fop->RelativeVertical *
                    _GPS_FEET_PER_METER));
        break;
      case UNITS_MIXED:
        u_dist = "kms";
        u_alt  = "feet";
        voc_dist = traffic[i].distance / 1000.0;
        voc_alt  = abs((int) (traffic[i].fop->RelativeVertical *
                    _GPS_FEET_PER_METER));
        break;
      case UNITS_METRIC:
      default:
        u_dist = "kms";
        u_alt  = "metres";
        voc_dist = traffic[i].distance / 1000.0;
        voc_alt  = abs((int) traffic[i].fop->RelativeVertical);
        break;
      }

      if (voc_dist < 1.0) {
        strcpy(how_far, "near");
      } else {
        if (voc_dist > 9.0) {
          voc_dist = 9.0;
        }
        snprintf(how_far, sizeof(how_far), "%u %s", (int) voc_dist, u_dist);
      }

      if (voc_alt < 100) {
        strcpy(elev, "near");
      } else {
        if (voc_alt > 500) {
          voc_alt = 500;
        }

        snprintf(elev, sizeof(elev), "%u hundred %s %s",
          (voc_alt / 100), u_alt,
          traffic[i].fop->RelativeVertical > 0 ? "above" : "below");
      }

      snprintf(message, sizeof(message),
                  "traffic %s distance %s altitude %s",
                  where, how_far, elev);

      traffic[i].fop->alert |= TRAFFIC_ALERT_VOICE;
      traffic[i].fop->timestamp = now();

      SoC->TTS(message);

      /* Speak up of one aircraft at a time */
      break;
    }
  }
}

void Traffic_setup()
{
  UpdateTrafficTimeMarker = millis();
  Traffic_Voice_TimeMarker = millis();
}

void Traffic_loop()
{
  if (settings->protocol == PROTOCOL_GDL90) {
    if (isTimeToUpdateTraffic()) {
      for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {

        if (Container[i].ID &&
            (ThisAircraft.timestamp - Container[i].timestamp) <= ENTRY_EXPIRATION_TIME) {
          if ((ThisAircraft.timestamp - Container[i].timestamp) >= TRAFFIC_VECTOR_UPDATE_INTERVAL)
            Traffic_Update(&Container[i]);
        } else {
          Container[i] = EmptyFO;
        }
      }

      UpdateTrafficTimeMarker = millis();
    }
  }

  if (isTimeToVoice()) {
    if (settings->voice != VOICE_OFF) {
      Traffic_Voice();
    }

    Traffic_Voice_TimeMarker = millis();
  }
}

void Traffic_ClearExpired()
{
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID && (now() - Container[i].timestamp) > ENTRY_EXPIRATION_TIME) {
      Container[i] = EmptyFO;
    }
  }
}

int Traffic_Count()
{
  int count = 0;

  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].ID) {
      count++;
    }
  }

  return count;
}

int traffic_cmp_by_distance(const void *a, const void *b)
{
  traffic_by_dist_t *ta = (traffic_by_dist_t *)a;
  traffic_by_dist_t *tb = (traffic_by_dist_t *)b;

  if (ta->distance >  tb->distance) return  1;
  if (ta->distance == tb->distance) return  0;
  if (ta->distance <  tb->distance) return -1;
}
