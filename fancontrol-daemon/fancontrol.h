/*
 * Copyright 2020 tobias.gierke@code-sourcery.de
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FANCONTROL_H
#define FANCONTROL_H

#define FANCONTROL_VERSION "1.0.0 (2020-03-27)"

enum fc_sensor_type {
    SENSOR_UNKNOWN = 0,
    SENSOR_MAINBOARD = 1<<0,
    SENSOR_HDD = 1<<1
};

typedef struct mb_sensor {
    char *chip_name;
    char *feature_name;
    char *subfeature_name;
} mb_sensor;

typedef struct hdd_sensor {
    char *device;
} hdd_sensor;

typedef struct temp_sensor {
      struct temp_sensor *next;
      enum fc_sensor_type type;
      union {
          mb_sensor mainboard;
          hdd_sensor hdd;
      };
} temp_sensor;

typedef struct fan_speed_mapping {
    struct fan_speed_mapping *next;
    int temperature;
    int fan_speed;
} fan_speed_mapping;

typedef struct thermal_zone 
{
   struct thermal_zone *next;
   char *name;
   temp_sensor *sensors;
   fan_speed_mapping *fan_speed_mapping;
} thermal_zone;

typedef struct fc_config {
    int min_fan_speed;
    int max_fan_speed;
    int loop_delay_millis;
    double smoothing_alpha;
    char *serial_device;
    int baud_rate;
    thermal_zone *thermal_zones;    
} fc_config;

/* RUNTIME STATE */

typedef struct temp_reading {    
    struct temp_reading *next;
    thermal_zone *thermal_zone;
    enum fc_sensor_type type;
    double value;
    union {
      mb_sensor mainboard;
      hdd_sensor hdd;          
    };
} temp_reading;

#endif
