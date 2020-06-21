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

#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sensors/sensors.h>
#include <atasmart.h>
#include "fancontrol.h"
#include <cjson/cJSON.h>
#include <signal.h>
#include <syslog.h>
#include <math.h>
#include <time.h>  
#include <systemd/sd-daemon.h>

#define DEFAULT_MIN_FAN_SPEED 40
#define DEFAULT_MAX_FAN_SPEED 100
#define DEFAULT_SMOOTHING_ALPHA 0.5
#define DEFAULT_LOOP_DELAY_MILLIS 500

// log message priorities
enum fc_prio {
    FC_PRIO_DEBUG,
    FC_PRIO_INFO,
    FC_PRIO_WARN,
    FC_PRIO_ERROR
};

// whether verbose debug output should be generated
static int debugMode = 0;

// whether we're running in daemon (forked) mode
static int is_daemon;

// file descriptor of serial device
// we're using to talk to the fan controller
static int fd;

// time in microseconds (1/1000000) for receiving/transmitting 
// one byte of data at the configured serial line rate
static int microsPerByte;

/**
 * Millisecond sleeps.
 * @param mseg sleep time in milliseconds.
 */
void msleep(int msec)
{
    struct timespec ts;

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    // I don't care about waking up early
    // so EINTR is ignored
    nanosleep(&ts, &ts);
}

/**
 * Write log message to syslog (when running
 * as a deamon) or stdout (otherwise).
 * 
 * @param prio priority of the message
 * @param format printf compatible format string
 * @param va arguments (if any) used by the format string
 */
void fc_log(enum fc_prio prio, const char *format, va_list va)
{    
    const char *prefix;
    int syslog_prio;
    switch(prio) {
        case FC_PRIO_DEBUG:
            syslog_prio = LOG_DEBUG;
            prefix = "DEBUG: %s";            
            break;
        case FC_PRIO_WARN:
            syslog_prio = LOG_WARNING;
            prefix = "WARNING: %s";            
            break;
        case FC_PRIO_ERROR:
            syslog_prio = LOG_ERR;
            prefix = "ERROR: %s";
            break;    
        case FC_PRIO_INFO:
            /* FALL-THROUGH */
        default:
            syslog_prio = LOG_INFO;            
            prefix = "INFO: %s";            
    }
            
    int len = strlen(format)+strlen(prefix); 
    char *newformat = calloc(1,len+2);
    
    const char *fmt;
    if ( newformat != NULL ) {
        snprintf(newformat,len,prefix,format);
        fmt=newformat;        
    } else {        
        fmt=format;        
    }
            
    if ( is_daemon ) {
        vsyslog(syslog_prio,fmt,va);
    } else {
        vprintf(fmt,va);
        printf("\n");
    }    
    
    if ( newformat != NULL ) {
        free(newformat);
    }    
}

/**
 * Write log message with priority FC_PRIO_DEBUG to syslog (when running
 * as a deamon) or stdout (otherwise).
 * 
 * @param format printf compatible format string
 * @param va arguments (if any) used by the format string
 */
void fc_log_debug(const char *format, ...)
{
    va_list myargs;
    
    if ( debugMode ) {
        va_start(myargs, format);
        fc_log(FC_PRIO_DEBUG,format,myargs);
        va_end(myargs);
    }
}

/**
 * Write log message with priority FC_PRIO_INFO to syslog (when running
 * as a deamon) or stdout (otherwise).
 * 
 * @param format printf compatible format string
 * @param va arguments (if any) used by the format string
 */
void fc_log_info(const char *format, ...)
{
    va_list myargs;
    
    va_start(myargs, format);
    fc_log(FC_PRIO_INFO,format,myargs);
    va_end(myargs);
}

/**
 * Write log message with priority FC_PRIO_WARN to syslog (when running
 * as a deamon) or stdout (otherwise).
 * 
 * @param format printf compatible format string
 * @param va arguments (if any) used by the format string
 */
void fc_log_warn(const char *format, ...)
{
    va_list myargs;
    
    va_start(myargs, format);
    fc_log(FC_PRIO_WARN,format,myargs);
    va_end(myargs);
}

/**
 * Write log message with priority FC_PRIO_ERROR to syslog (when running
 * as a deamon) or stdout (otherwise).
 * 
 * @param format printf compatible format string
 * @param va arguments (if any) used by the format string
 */
void fc_log_err(const char *format, ...)
{
    va_list myargs;
    
    va_start(myargs, format);
    fc_log(FC_PRIO_ERROR,format,myargs);
    va_end(myargs);
}

/**
 * Maps a serial line bitrate into the corresponding
 * POSIX compliant value.
 * 
 * @param baud_rate input baud rate
 * @return POSIX value suitable for use with cfsetispeed() / cfsetospeed() or -1 if no such mapping exists
 */
int fc_map_baudrate(int baud_rate) 
{    
    switch(baud_rate) 
    {
        case 1200:  return   B1200;
        case 2400:  return   B2400;
        case 4800:  return   B4800;
        case 9600:  return   B9600;
        case 19200: return  B19200;
        case 38400: return  B38400;
        case 57600: return  B57600;
        case 115200:return B115200;
        default:
            fc_log_err("Invalid baud rate %d",baud_rate);
            return -1;
    }
}

/**
 * Turns this application into a daemon process
 * by forking twice, closing all file descriptors,
 * becoming a session leader etc.
 */
static void daemonize()
{
    pid_t pid;

    /* Fork off the parent process */
    pid = fork();

    /* An error occurred */
    if (pid < 0) {
        exit(EXIT_FAILURE);
    }

    if (pid > 0) {
        // this is the parent process, terminate.
        exit(EXIT_SUCCESS);
    }

    // THIS IS THE CHILD PROCESS
    if (setsid() < 0) {
        exit(EXIT_FAILURE);
    }

    /* Catch, ignore and handle signals */
    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);

    /* Fork off for the second time*/
    pid = fork();

    /* An error occurred */
    if (pid < 0) {
        exit(EXIT_FAILURE);
    }

    if (pid > 0) {
        // this is the parent process, terminate
        exit(EXIT_SUCCESS);
    }

    /* Set new file permissions */
    umask(0);

    /* Change the working directory to the root directory */
    /* or another appropriated directory */
    chdir("/tmp");

    /* Close all open file descriptors */
    int x;
    for (x = sysconf(_SC_OPEN_MAX); x>=0; x--)
    {
        close (x);
    }

    /* Open the log file */
    openlog("fancontrol", LOG_PID, LOG_USER);
}

/**
 * Releases any memory allocated with a temp_sensor struct.
 * 
 * @param sensor structure to free
 */
void fc_free_sensor(temp_sensor *sensor) {
     
    if ( sensor->type == SENSOR_MAINBOARD ) {
        free(sensor->mainboard.chip_name);    
        free(sensor->mainboard.feature_name);        
        free(sensor->mainboard.subfeature_name);        
    } else if ( sensor->type == SENSOR_HDD ) {
        free(sensor->hdd.device);          
    } else {
      fc_log_err("free_sensor(): Unhandled sensor type %d", sensor->type);           
    }
    free(sensor);    
}

/**
 * Releases any memory allocated with a fan_speed_mapping struct.
 * 
 * @param mapping structure to free
 */
void fc_free_fan_speed_mapping(fan_speed_mapping *mapping) {
    free(mapping);
}

/**
 * Releases any memory allocated with a thermal_zone struct.
 * 
 * @param zone structure to free
 */
void fc_free_thermal_zone(thermal_zone *zone) 
{
    free(zone->name);    
    if ( zone->sensors != NULL ) {
        temp_sensor *current = zone->sensors;                    
        while ( current != NULL ) {            
            temp_sensor *next = current->next;            
            fc_free_sensor(current);
            current = next;
        } 
    }
    if ( zone->fan_speed_mapping != NULL ) {
        fan_speed_mapping *current = zone->fan_speed_mapping;                    
        while ( current != NULL ) {            
            fan_speed_mapping *next = current->next;            
            fc_free_fan_speed_mapping(current);
            current = next;
        }        
    }
    free(zone);
}

/**
 * Releases any memory allocated with a fc_config struct.
 * 
 * @param config structure to free
 */
void fc_free_config(fc_config *config) {
    
    thermal_zone *current=config->thermal_zones;
    while ( current != NULL ) {
         thermal_zone *next=current->next;
         fc_free_thermal_zone(current);
         current = next;
    }
    if ( config->serial_device != NULL ) {
      free(config->serial_device);    
    }
    free(config);
}

/**
 * Parse JSON for the single sensor..
 * 
 * @param json current JSON parse state
 * @return pointer to parsed configuration or NULL on error
 */
temp_sensor *fc_parse_sensor(cJSON *json) {
    
    if ( ! cJSON_IsObject(json) ) {
      fc_log_err("fc_parse_sensor(): expected an object()");           
      return NULL;
    }
    
    enum fc_sensor_type stype = SENSOR_UNKNOWN;
    // find sensor type
    cJSON *current = json->child;
    while ( current != NULL && stype == SENSOR_UNKNOWN ) 
    {
        if ( cJSON_IsString(current) && ! strcmp(current->string,"type") ) {
            if ( ! strcmp(current->valuestring,"mainboard") ) {
                stype = SENSOR_MAINBOARD;
            } else if ( ! strcmp(current->valuestring,"hdd") ) {
                stype = SENSOR_HDD;
            } else {
                fc_log_err("fc_parse_sensor(): Unhandled sensor type %s", current->valuestring);                 
            }
        }
        current = current->next;
    }
    
    if ( stype == SENSOR_UNKNOWN ) {
      fc_log_err("fc_parse_sensor(): Could not determine sensor type");           
      return NULL;
    }
    
    temp_sensor *result = calloc(1,sizeof(temp_sensor));
    if ( result == NULL ) {
      fc_log_err("fc_parse_sensor(): calloc() failed");           
      return NULL;
    }  
    result->type=stype;
    
    // parse type attributes
    current = json->child; 
    while ( current != NULL ) 
    {
        if ( ! cJSON_IsString(current) ) {
            fc_log_err("fc_parse_sensor(): Unexpected JSON node type");                
            fc_free_sensor(result);
            return NULL;            
        }
        if ( ! strcmp(current->string,"type") ) {
            current = current->next;
            continue;
        }
        switch(stype) 
        {
            case SENSOR_MAINBOARD:
                if ( ! strcmp(current->string,"chip") ) {
                    result->mainboard.chip_name = strdup(current->valuestring);
                } else if ( ! strcmp(current->string,"feature") ) {
                    result->mainboard.feature_name = strdup(current->valuestring);
                } else if ( ! strcmp(current->string,"subfeature") ) {
                    result->mainboard.subfeature_name = strdup(current->valuestring);
                } else {
                    fc_log_err("fc_parse_sensor(): Unhandled mainboard sensor attribute %s", current->valuestring);    
                    fc_free_sensor(result);
                    return NULL;                            
                }
                break;
            case SENSOR_HDD:
                if ( ! strcmp(current->string,"device") ) {                    
                    result->hdd.device=strdup(current->valuestring);
                } else {
                    fc_log_err("fc_parse_sensor(): Unhandled hdd sensor attribute %s", current->valuestring);                     
                    fc_free_sensor(result);
                    return NULL;                      
                }
                break;
            default:
                fc_log_err("fc_parse_sensor(): Unhandled sensor type %d", stype);             
                fc_free_sensor(result);
                return NULL;  
        }
        current = current->next;
    }
    return result;    
}

/**
 * Parse JSON for fan_speed_mapping.
 * 
 * @param json current JSON parse state
 * @return pointer to parsed configuration or NULL on error
 */
fan_speed_mapping *fc_parse_fan_speed_mapping(cJSON *json) 
{
    if ( ! cJSON_IsNumber(json) ) {
      fc_log_err("fc_parse_fan_speed_mapping(): expected a number but got %d",json->type);           
      return NULL;
    }
    
    cJSON *attr = json;
    
    fan_speed_mapping *result = NULL;
    fan_speed_mapping *previous = NULL;
    while( attr != NULL ) 
    {
        fan_speed_mapping *current = calloc(1,sizeof(fan_speed_mapping));        
        if ( current == NULL ) 
        {
            fc_log_err("fc_parse_fan_speed_mapping(): calloc() failed");               
            goto error;
        }
        current->temperature = atoi(attr->string);
        if ( current->temperature < 0 || current->temperature>100 ) {
            fc_log_err("fc_parse_fan_speed_mapping(): invalid temperature %d, must be 0 <= temp <= 100",current->temperature);               
            goto error;            
        }
        current->fan_speed = attr->valueint;     
        if ( current->fan_speed < 0 || current->fan_speed>100 ) {
            fc_log_err("fc_parse_fan_speed_mapping(): invalid fan speed %d, must be 0 <= fan_speed <= 100",current->fan_speed);               
            goto error;            
        }      
        if ( result == NULL ) {
            result = current;
        }
        if ( previous != NULL ) {
            previous->next=current;
        }
        previous = current;
        attr = attr->next;
    }    
    return result;
    
error:
    for ( fan_speed_mapping *tmp = result ; tmp != NULL ; ) 
    {
        fan_speed_mapping *next = tmp->next;
        fc_free_fan_speed_mapping(tmp);
        tmp = next;
    }
    return NULL;
}

/**
 * Parse JSON for a thermal zone.
 * 
 * @param json current JSON parse state
 * @return pointer to parsed configuration or NULL on error
 */
thermal_zone* fc_parse_thermal_zone(cJSON *json) {

    if ( ! cJSON_IsObject(json) ) {
      fc_log_err("parse_thermal_zone(): expected an object()");           
      return NULL;
    }
    
    thermal_zone *result=calloc(1,sizeof(thermal_zone));
    if ( result == NULL ) {
      fc_log_err("parse_thermal_zone(): calloc() 1 failed");           
      return NULL;
    }
    cJSON *attr = json->child;
    while( attr != NULL ) 
    {
        if ( ! strcmp(attr->string,"name") ) {
            result->name = strdup(attr->valuestring);    
        } 
        else if ( ! strcmp(attr->string,"fan_speed_mapping") ) 
        {
            result->fan_speed_mapping = fc_parse_fan_speed_mapping(attr->child);
            if ( result->fan_speed_mapping == NULL ) {
                goto error;               
            }          
        } else if ( ! strcmp(attr->string,"sensors") ) {
            cJSON *sensor=attr->child;
            
            if ( cJSON_IsObject(sensor) ) 
            {
                result->sensors = fc_parse_sensor(sensor);                
            } 
            else if ( cJSON_IsArray(sensor) ) 
            {
                sensor = sensor->child;
                temp_sensor *previous = NULL;
                while ( sensor != NULL ) {
                    temp_sensor *s = fc_parse_sensor(sensor);
                    if ( previous != NULL ) {
                        previous->next = s;    
                    }
                    previous = s;
                    sensor = sensor->next;
                }
            } else {
                fc_log_err("parse_thermal_zone(): Expected an JSON array or Object for 'sensors' but got %d",sensor->type);               
                goto error;              
            }
        } else {
            fc_log_err("parse_thermal_zone(): Unexpected JSON attribute %s",attr->string);               
            goto error;
        }
        
        attr = attr->next;
    }
    return result;
    
error:
    fc_free_thermal_zone(result);
    return NULL;
}

/**
 * Prints the current configuration as JSON to stdout.
 * 
 * @param config configuration to print
 */
void fc_debug_print_config(fc_config *config) {
 
    printf("{\n");
    printf("    \"serial_device\" : \"%s\",\n",config->serial_device);
    printf("    \"baud_rate\" : %d,\n",config->baud_rate);
    printf("    \"min_fan_speed\" : %d,\n",config->min_fan_speed);
    printf("    \"max_fan_speed\" : %d,\n",config->max_fan_speed);
    
    printf("    \"thermal_zones\" : [\n");
    thermal_zone *zone = config->thermal_zones;
    while ( zone != NULL ) {
    
        printf("        {\n");      
        
            printf("            \"name\" : \"%s\",\n",zone->name);
            printf("            \"sensors\" :[\n");
            temp_sensor *sensor = zone->sensors;
            while( sensor != NULL ) {
                
                printf("                {");
                switch(sensor->type) {
                    case SENSOR_MAINBOARD:
                        printf("                \"type\" : \"mainboard\",\n");                    
                        printf("                \"chip\" : \"%s\",\n",sensor->mainboard.chip_name);                    
                        printf("                \"feature\" : \"%s\",\n",sensor->mainboard.feature_name);                    
                        printf("                \"subfeature\" : \"%s\"\n",sensor->mainboard.subfeature_name);                    
                        break;
                    case SENSOR_HDD:
                        printf("                \"type\" : \"hdd\",\n");                    
                        printf("                \"device\" : \"%s\"\n",sensor->hdd.device);                    
                        break;
                    default:
                        printf("                \"type\" : \"unknown\"\n");
                }
                            
                printf("            }");
                
                sensor = sensor->next;
                if ( sensor != NULL ) {
                    printf(",\n");
                }
            }
            printf("],\n"); // end of 'sensors' array
            
            printf("            \"fan_speed_mappings\" : {\n");
            fan_speed_mapping *mapping = zone->fan_speed_mapping;
            while( mapping != NULL ) {
                printf("                \"%d\" : %d",mapping->temperature,mapping->fan_speed);
                mapping = mapping->next;
                if ( mapping != NULL ) {
                    printf(",\n");
                } else {
                    printf("\n");                
                }                
            }
            printf("            }\n");
        printf("        }");        
        
        zone = zone->next;
        if ( zone != NULL ) {
            printf(",\n");
        } else {
            printf("\n");            
        }
    }
    printf("]}\n"); // end of 'thermal_zones' array
}

/**
 * Loads application configuration from a JSON file.
 * 
 * @param jsonFile file to load
 * @return parsed configuration or NULL on error
 */
fc_config *fc_load_configuration(char *jsonFile) 
{
    fc_log_info("Loading configuration from %s",jsonFile);
    
    FILE *file = fopen(jsonFile,"r");
    if ( file == NULL ) {
      fc_log_err("load_configuration(): failed to open configuration file %s", jsonFile); 
      return NULL;    
    }
    
    char json_str[2048];
    int json_str_len = fread(&json_str,1,sizeof(json_str),file);
    fclose(file);
    if ( json_str_len < 0 || json_str_len == sizeof(json_str)) {
      fc_log_err("load_configuration(): reading configuration file %s failed", jsonFile); 
      return NULL;    
    }   
    json_str[json_str_len] = 0;
    
    cJSON *json = cJSON_Parse(json_str);
    if ( json == NULL ) {
        fc_log_err("load_configuration(): Failed to parse JSON file %s - %s", jsonFile, cJSON_GetErrorPtr() );         
        return NULL;
    }
        
    fc_config *result = calloc(1,sizeof(fc_config));
    
    if ( result != NULL ) 
    {
        result->smoothing_alpha = DEFAULT_SMOOTHING_ALPHA;
        result->min_fan_speed = DEFAULT_MIN_FAN_SPEED;
        result->max_fan_speed = DEFAULT_MAX_FAN_SPEED;
        result->loop_delay_millis = DEFAULT_LOOP_DELAY_MILLIS;
        
        if ( ! cJSON_IsObject(json) || json->child == NULL ) 
        {
            fc_log_err("load_configuration(): Failed to parse JSON file %s - expected an object with at least one child",jsonFile);            
            goto error;
        }
        cJSON *child = json->child;
        while ( child != NULL ) {
            if ( ! strcmp( child->string, "thermal_zones") ) 
            {
                
                if ( ! cJSON_IsArray(child) ) {
                    fc_log_err("load_configuration(): Failed to parse JSON file %s - 'thermal_zones' needs to be an array",jsonFile);            
                    goto error;                        
                }
                
                thermal_zone *previousZone = NULL;                    
                cJSON *zone = child->child;     
                while ( zone != NULL ) {
                    thermal_zone *currentZone = fc_parse_thermal_zone(zone);
                    if ( currentZone == NULL ) {
                        fc_log_err("load_configuration(): Failed to parse thermal zone");
                        goto error;                          
                    }
                    
                    if ( result->thermal_zones == NULL ) {
                        result->thermal_zones=currentZone;
                    }
                    
                    if ( previousZone != NULL ) {
                        previousZone->next = currentZone;
                    }
                    previousZone = currentZone;
                    zone = zone->next;
                }         
            } else if ( ! strcmp( child->string, "serial_device") ) {
                result->serial_device=strdup(child->valuestring);
            } else if ( ! strcmp( child->string, "baud_rate") ) {
                result->baud_rate=child->valueint;               
                if ( fc_map_baudrate(result->baud_rate) == -1 ) {
                    fc_log_err("load_configuration(): Failed to parse JSON file %s - invalid baud rate %d",jsonFile,result->baud_rate);            
                    goto error;                       
                }
            } else if ( ! strcmp( child->string, "loop_delay_millis") ) {
                result->loop_delay_millis=child->valueint;   
                if ( result->loop_delay_millis < 100 ) {
                    fc_log_err("load_configuration(): Invalid loop_delay_millis value %d, must be at least 100 ms",jsonFile,result->loop_delay_millis);            
                    goto error;                           
                }                
            } else if ( ! strcmp( child->string, "smoothing_alpha") ) {
                result->smoothing_alpha = child->valuedouble;   
                if ( result->smoothing_alpha <= 0.0 || result->smoothing_alpha >= 1.0 ) {
                    fc_log_err("load_configuration(): Invalid smoothing_alpha value %f",jsonFile,result->smoothing_alpha);            
                    goto error;                           
                }
            } else if ( ! strcmp( child->string, "max_fan_speed") ) {
                result->max_fan_speed=child->valueint;            
                if ( result->max_fan_speed < 0 || result->max_fan_speed > 100 ) {
                    fc_log_err("load_configuration(): Invalid max_fan_speed value %d, must be 0<= value <=100",jsonFile,result->max_fan_speed);            
                    goto error;                           
                }                
            } else if ( ! strcmp( child->string, "min_fan_speed") ) {
                result->min_fan_speed=child->valueint;
                if ( result->min_fan_speed < 0 || result->min_fan_speed > 100 ) {
                    fc_log_err("load_configuration(): Invalid min_fan_speed value %d, must be 0<= value <=100",jsonFile,result->min_fan_speed);            
                    goto error;                           
                }                     
            } else {
                fc_log_err("load_configuration(): Failed to parse JSON file %s - unexpected child '%sd'",jsonFile,child->string);            
                goto error;                
            }
            child = child->next;
        }         
    } else {
        fc_log_err("load_configuration(): Failed to allocate memorys");
    }    
    
    if ( result->min_fan_speed > result->max_fan_speed ) {
        fc_log_err("load_configuration(): Failed to parse JSON file %s - min_fan_speed %d exceeds max_fan_speed %d",jsonFile,result->min_fan_speed,result->max_fan_speed);
        goto error;
    }   
    
    if ( result->baud_rate == 0 ) {
        fc_log_err("load_configuration(): Failed to parse JSON file %s - file contains no baud rate",jsonFile);
        goto error;
    }    
    
    if ( result->serial_device == NULL ) {
        fc_log_err("load_configuration(): Failed to parse JSON file %s - file contains no serial device",jsonFile);
        goto error;
    }
    
    cJSON_Delete(json);    
    
    return result;
    
error:
    cJSON_Delete(json);    
    free(result);
    return NULL;
}

/**
 * Apply configuration to serial device.
 * 
 * @param config configuration to apply
 * @param fd file descriptor of serial device
 * @return 0 on error, 1 on success
 */
int fc_set_interface_attribs(int fd, fc_config *config)
{
    struct termios tty;
    
    memset(&tty,0,sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0)
    {
        fc_log_err("fc_set_interface_attribs(): error %d from tcgetattr", errno);
        return 0;
    }
    
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    
    tty.c_cflag &= ~CSIZE; // 8 bits per byte (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 50;    // Wait for up to 5s (50 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 1;
    
    speed_t rate = fc_map_baudrate(config->baud_rate);
    cfsetispeed(&tty, rate);
    cfsetospeed(&tty, rate);
    
    microsPerByte = 1000000 / (config->baud_rate/9); // 8N1 -> 9 bits per data byte
    fc_log_debug("fc_set_interface_attribs(): time for transmitting/receiving one byte: %d microseconds",microsPerByte);

    if (tcsetattr (fd, TCSAFLUSH, &tty) != 0)
    {
        fc_log_err("fc_set_interface_attribs(): error %d from tcsetattr", errno);
        return 0;
    }
    return 1;
}

/**
 * Open & configure serial device to fan controller.
 * @param config configuration to used
 * @return 0 on error, 1 on success
 */
int fc_ser_open(fc_config *config) 
{
    fd = open(config->serial_device, O_RDWR | O_NOCTTY | O_NDELAY );    
    if (fd < 0)
    {
        fc_log_err ("fc_ser_open(): error %d opening %s: %s", errno, config->serial_device, strerror (errno));
        return 0;
    }

    if ( ! fc_set_interface_attribs(fd,config) ) {  
        close(fd);
        return 0;
    }    
    return 1;
}

/**
 * Write null-terminated string to serial device.
 * @param s string to write
 * @return 0 on error, 1 on sucess
 */
int fc_ser_write(unsigned char *s) {
    
    char buf[1];
    
    char *ptr = s;
    while ( *ptr ) {
        buf[0] = *ptr++;
        int res = write(fd, buf,1);           
        if ( res < 1 ) {
          return 0;    
        }
        usleep(microsPerByte*10);        
    }
    return 1;
}

/**
 * Read line from serial device (string will be null-terminated).
 * 
 * @param buf buffer where string will be stored, must be able to hold at least 2 bytes.
 * @param bufRemaining number of bytes available in the buffer
 * @return number of bytes written to the buffer (including terminating zero byte)
 */
int fc_ser_readline(char *buf, int bufRemaining) {
        
    int writePtr = 0;
    char tmp;
    int retryCount = 10;
    int result;
    
    fc_log_debug("fc_ser_readline(): Reading response");        
    
    buf[0] = 0;
    while (bufRemaining > 1 && (result = read(fd, &tmp, 1)) >= 0 ) 
    {       
        if ( result == 0 ) {
            retryCount--;
            if ( retryCount == 0 ) {
                fc_log_err("fc_ser_readline(): Didn't receive byte after 10 timeouts");
                break;
            }            
            usleep(microsPerByte);
            continue;
        }
        retryCount = 10;
        buf[writePtr++] = tmp;
        bufRemaining--;                
        if ( tmp == 0x0a ) {    
          writePtr--;
          break;
        }                                               
    }   
    buf[writePtr++] = 0;    
    return writePtr;
}

/**
 * Releases any memory allocated with a temperature reading.
 * 
 * @param s structure to free
 */
void fc_free_temp_readings(temp_reading *s) {
 
    temp_reading *current = s;
    while ( current != NULL ) {
        temp_reading *next=current->next;
        switch(current->type) {
            case SENSOR_MAINBOARD:
                free(current->mainboard.chip_name);
                free(current->mainboard.feature_name);
                free(current->mainboard.subfeature_name);
                break;
            case SENSOR_HDD:
                free(current->hdd.device);
                break;
            default:
                fc_log_err("fc_free_temp_readings(): Unhandled sensor type %d",current->type);
        }
        free(current);
        current = next;
    }    
}

/**
 * Read temperature of a mainboard sensor.
 * 
 * @param sensor temperature sensor to read
 * @return temperature reading or NULL on error
 */
temp_reading* fc_query_mainboard_sensor(mb_sensor *sensor) 
{        
    sensors_chip_name const * chip_name;    
    int chip_id = 0;                    
    while ((chip_name = sensors_get_detected_chips(0, &chip_id)) != 0) 
    {        
        sensors_feature const *feature;
        int feature_id = 0;

        if ( ! strcmp(chip_name->prefix,sensor->chip_name) ) 
        {
            while ( (feature = sensors_get_features(chip_name, &feature_id)) != 0) 
            {
                sensors_subfeature const *subfeature;
                int subfeature_id = 0;

                if ( ! strcmp(feature->name,sensor->feature_name) )
                {
                    while ((subfeature = sensors_get_all_subfeatures(chip_name, feature, &subfeature_id)) != 0) 
                    {                   
                        if (  ! strcmp(subfeature->name,sensor->subfeature_name) ) 
                        {        
                            if (subfeature->flags & SENSORS_MODE_R) 
                            {
                                double sensor_reading;                      
                                int rc = sensors_get_value(chip_name, subfeature->number, &sensor_reading);
                                if (rc < 0) {
                                    fc_log_warn("fc_query_mainboard_sensor(): failed to read value for sensor %s | %s | %s",
                                                sensor->chip_name,sensor->feature_name,sensor->subfeature_name);
                                    fc_log_warn("fc_query_mainboard_sensor(): err: %d",rc);
                                    return NULL;
                                }
                                temp_reading *current = calloc(1,sizeof(temp_reading));
                                if ( current == NULL ) {
                                    fc_log_err("fc_query_mainboard_sensor(): calloc() failed");
                                    return NULL;
                                }
                                current->type = SENSOR_MAINBOARD;                                            
                                current->mainboard.chip_name = strdup(sensor->chip_name);
                                current->mainboard.feature_name = strdup(sensor->feature_name);                
                                current->mainboard.subfeature_name = strdup(sensor->subfeature_name);                
                                current->value = sensor_reading;
                                return current;
                            } 
                            fc_log_err("fc_query_mainboard_sensor(): Sensor %s | %s | %s is not readable",sensor->chip_name,sensor->feature_name,sensor->subfeature_name);        
                            return NULL;
                        }
                    }
                }        
            }
        }
    }
    fc_log_err("fc_query_mainboard_sensor(): Failed to find sensor %s | %s | %s",sensor->chip_name,sensor->feature_name,sensor->subfeature_name);        
    return NULL;
}

/**
 * Returns a bit-mask indicating which types
 * of sensors are present in a list of thermal zones.
 * 
 * @param zone start of thermal zones list to check
 * @return bit-mask of all sensor types present in the list
 */
int get_sensor_types(thermal_zone *zone) 
{
    int types = 0;
    temp_sensor *sensor= zone->sensors;
    while ( sensor != NULL ) {
        types |= sensor->type;
        sensor = sensor->next;
    }    
    return types;
}

/**
 * Read temperature of all mainboard sensors for a thermal zone (if any)
 * OR return readings for all available sensors.
 * 
 * @param zone thermal zone to check mainboard sensors for or NULL to return readings for all sensors.
 * @return temperature readings or NULL on error
 */
temp_reading* fc_query_mainboard_sensors(thermal_zone *zone) 
{
    temp_reading* result = NULL;
    
    if ( zone != NULL ) 
    {
        while ( zone != NULL ) 
        {
            temp_sensor *sensor = zone->sensors;
            while ( sensor != NULL ) 
            {
                if ( sensor->type == SENSOR_MAINBOARD ) 
                {
                    temp_reading* r = fc_query_mainboard_sensor(&sensor->mainboard);
                    if ( r == NULL ) {
                        fc_log_err("fc_query_mainboard_sensors(): Failed to query sensor %s  | %s | %s",sensor->mainboard.chip_name,sensor->mainboard.feature_name,sensor->mainboard.subfeature_name);
                        fc_free_temp_readings(result);
                        return NULL;
                    }
                    r->thermal_zone=zone;
                    if ( result == NULL ) {
                        result = r;
                    } else {
                        r->next=result;
                        result=r;
                    }
                }
                sensor = sensor->next;
            }
            zone = zone->next;
        }        
        return result;
    }
    
    /*
     * Discover sensors
     */
    
    char buffer[200];
    sensors_chip_name const * chip_name;    
    int chip_id = 0;                    
    while ((chip_name = sensors_get_detected_chips(0, &chip_id)) != 0) 
    {
        sensors_feature const *feature;
        int feature_id = 0;

        while ( (feature = sensors_get_features(chip_name, &feature_id)) != 0) 
        {
            sensors_subfeature const *subfeature;
            int subfeature_id = 0;

            while ((subfeature = sensors_get_all_subfeatures(chip_name, feature, &subfeature_id)) != 0) 
            {                   
                if (subfeature->flags & SENSORS_MODE_R) 
                {
                    double sensor_reading;                      
                    int rc = sensors_get_value(chip_name, subfeature->number, &sensor_reading);
                    if (rc < 0) {
                        fc_log_warn("fc_query_mainboard_sensors(): failed to read value for features %s, subfeature %d : %s (%d) =",feature->name,subfeature_id,subfeature->name,subfeature->number);
                        fc_log_warn("fc_query_mainboard_sensors(): err: %d",rc);
                    } else {
                        temp_reading *current = calloc(1,sizeof(temp_reading));
                        if ( current == NULL ) {
                            fc_log_err("fc_query_mainboard_sensor(): calloc() failed");
                            fc_free_temp_readings(result);
                            return NULL;
                        }
                        current->type = SENSOR_MAINBOARD;
                        
                        // int sensors_snprintf_chip_name(char *str, size_t size, const sensors_chip_name *chip);
                        buffer[0] = 0;
                        if ( sensors_snprintf_chip_name(buffer,sizeof(buffer)-1,chip_name) < 0 ) {
                            fc_log_err("fc_query_mainboard_sensor(): failed to print chip name");
                            fc_free_temp_readings(result);
                            return NULL;                            
                        }
                        
                        current->mainboard.chip_name = strdup(buffer);
                        current->mainboard.feature_name = strdup(feature->name);                
                        current->mainboard.subfeature_name = strdup(subfeature->name);                
                        current->value = sensor_reading;
                        
                        if ( result == NULL ) {
                            result = current;
                        } else {
                            current->next = result;
                            result = current;                    
                        }                             
                    }
                }           
            }
        }
    }        
    return result;
}

/**
 * Release all memory allocated for a temperature reading.
 * @param s temperature reading to release
 */
void fc_release(temp_reading *s) 
{
    switch( s->type ) 
    {
        case SENSOR_MAINBOARD:
            free( s->mainboard.chip_name);
            free( s->mainboard.feature_name );
            free( s->mainboard.subfeature_name );                
            break;
        case SENSOR_HDD:
            free( s->hdd.device );
            break;
        default:
            fc_log_err("release(): Unhandled case: %d", s->type);            
    }    
}

/**
 * Logs a temperature reading with log level DEBUG.
 * @param s temperature reading to log
 */
void fc_print_reading(temp_reading *s) 
{
    switch( s->type ) 
    {
        case SENSOR_MAINBOARD:
            fc_log_debug("mainboard: %s | %s | %s = %f", s->mainboard.chip_name, s->mainboard.feature_name, s->mainboard.subfeature_name, s->value);               
            break;
        case SENSOR_HDD:
            fc_log_debug("HDD: %s = %f",s->hdd.device,s->value);               
            break;
        default:
            fc_log_err("print_reading(): Unhandled case: %d", s->type);            
    }
}

/**
 * Tries to read hard-disk temperature using SMART commands.
 * 
 * Devices that are currently sleeping (powered-down) will
 * report a temperature of 0 degrees.
 * 
 * 
 * @param device_path HDD device to get temperature fork
 * @return temperature reading or NULL on error.
 */
temp_reading* fc_query_hdd_sensor(char *device_path) 
{
    temp_reading* result = NULL;
    SkDisk *d;
    struct stat statbuf;
    SkBool awake;    
                
    if ( sk_disk_open(device_path, &d) >= 0) 
    {
        if ( sk_disk_check_sleep_mode(d,&awake) == 0 ) 
        {        
            if ( awake ) 
            {
                if ( sk_disk_smart_read_data(d) >= 0) 
                {
                    uint64_t mkelvin;        
                    if ( sk_disk_smart_get_temperature(d, &mkelvin) >= 0) 
                    {
                        result = calloc(1,sizeof(temp_reading));
                        if ( result != NULL ) 
                        {
                            result->type = SENSOR_HDD;
                            result->hdd.device = strdup(device_path);
                            result->value = mkelvin/1000.0 - 273.15;                                                 
                        } else {                        
                            fc_log_err("fc_query_hdd_sensors(): calloc() failed");                                       
                        }                        
                    } else {
                        fc_log_err("fc_query_hdd_sensors(): Failed to get temperature: %s", strerror(errno));
                    }
                } else {
                    fc_log_err("fc_query_hdd_sensors(): Failed to read SMART data: %s", strerror(errno));
                }      
            } else {
                fc_log_debug("fc_query_hdd_sensors(): Skipping sleeping device %s", device_path);        
                result = calloc(1,sizeof(temp_reading));
                if ( result != NULL ) 
                {
                    result->type = SENSOR_HDD;
                    result->hdd.device = strdup(device_path);
                    result->value = 0;                                                 
                } else {                        
                    fc_log_err("fc_query_hdd_sensors(): calloc() failed");                                       
                }                
            }
        } else {
            fc_log_err("fc_query_hdd_sensors(): Failed to check sleep mode for %s - %s", device_path, strerror(errno));                
        }            
        sk_disk_free(d);        
    } else {
        fc_log_err("fc_query_hdd_sensors(): Failed to open disk %s: %s", device_path, strerror(errno));            
    }        
    return result;       
}

/**
 * Get temperature readings for all hard-disk sensors in a 
 * given thermal zone.
 * 
 * @param zone thermal zone
 * @return temperature readings or NULL on error
 */
temp_reading* fc_query_hdd_sensors(thermal_zone *zone) 
{
    temp_reading *result = NULL;
    if ( zone == NULL ) {
        // TODO: discover block devices in a more thorough way?
        char *devices[] = { "/dev/sda", "/dev/sdb", "/dev/sdc", "/dev/sdd", "/dev/sde", "/dev/sdf" };
        for ( int i = 0 ; i < 6 ; i++ ) {
            temp_reading *r = fc_query_hdd_sensor(devices[i]);
            if ( r == NULL ) {
                fc_free_temp_readings(result);
                return NULL;
            }            
            if ( result == NULL ) {
                result = r;
            } else {
                r->next=result;
                result=r;
            }            
        }        
    } else {
        temp_sensor *sensor = zone->sensors;
        while ( sensor != NULL ) 
        {
            if ( sensor->type == SENSOR_HDD ) {
                temp_reading *r = fc_query_hdd_sensor(sensor->hdd.device);
                if ( r == NULL ) {
                    fc_free_temp_readings(result);
                    return NULL;
                }
                r->thermal_zone = zone;
                if ( result == NULL ) {
                    result = r;
                } else {
                    r->next=result;
                    result = r;
                }
            }        
            sensor = sensor->next;
        }        
    }
    return result;    
}

/**
 * Get temperature readings for all sensors in a 
 * given thermal zone.
 * 
 * @param zone thermal zone
 * @return temperature readings or NULL on error
 */
temp_reading *fc_read_temps(thermal_zone *zone) {
        
    temp_reading *result = NULL;
    if ( zone == NULL ) {
        // discover sensors
        result = fc_query_mainboard_sensors(NULL);    
        temp_reading *temp = fc_query_hdd_sensors(NULL);        
        if ( temp != NULL ) {
            if ( result == NULL ) {
                result = temp;
            } else {
                temp->next = result;
                result = temp;
            }
        }
        return result;
    }
    
    int sensor_types = get_sensor_types(zone);
    if ( sensor_types & SENSOR_MAINBOARD ) 
    {
        // we've got at least one mainboard temp sensor to read
        temp_reading *temp = fc_query_mainboard_sensors(zone);    
        if ( temp == NULL ) {
            fc_log_err("fc_read_temps(): Failed to read mainboard sensor");
            fc_free_temp_readings(result);
            return NULL;
        }                
        if ( result == NULL ) {
            result = temp;
        } else {
            temp->next=result;
            result=temp;
        }                
    }
    if ( sensor_types & SENSOR_HDD ) {
        // we've got at least one HDD temperature sensor
        temp_reading *temp = fc_query_hdd_sensors(zone);
        if ( temp == NULL ) {
            fc_log_err("fc_read_temps(): Failed to read HDD sensor");
            fc_free_temp_readings(result);
            return NULL;
        }              
        if ( result == NULL ) {
            result = temp;
        } else {
            temp->next=result;
            result=temp;
        }              
    }    
    return result;
}

/**
 * Send command to fan controller to change the fan speed.
 * 
 * @param fanSpeed fan speed in percent
 */
void send_fan_speed(double fanSpeed) {
    
    char buffer[50];
    snprintf(buffer,sizeof(buffer),"set %d\r",(int) fanSpeed);
    
    if ( fc_ser_write(buffer) ) {
        buffer[0]=0;
        int respSize = fc_ser_readline(buffer,sizeof(buffer));            
        fc_log_debug("send_fan_speed: response >%s<\n",&buffer[0]);             
        if ( respSize < 2 || buffer[0] != 'o' || buffer[1] != 'k' ) {
            fc_log_err("send_fan_speed(): no or unexpected response from fan controller: >%s<",&buffer[0]);
        }
    } else {
        fc_log_err("send_fan_speed(): serial write failed");
    }
}

int main(int argc,char **argv) 
{
    is_daemon = 0;
        
    // parse cmdline arguments
    
    char *configFileLocation = NULL;
    int fork = 0;    
    for ( int i = 1 ; i < argc ; i++ ) 
    {
        if ( ! strcmp(argv[i],"--h" ) || ! strcmp(argv[i],"--help" ) || ! strcmp(argv[i],"-help" )) {  
            fc_log_info("HPE Microserver System Fan Controller, version %s", FANCONTROL_VERSION);
            fc_log_info("");
            fc_log_info("-c|--config <JSON configuration file>");
            fc_log_info(" configuration file to use");
            fc_log_info("-h|--help|-help");
            fc_log_info("    prints this message");
            fc_log_info("-f|--daemon");
            fc_log_info("    run in background");            
            fc_log_info("-v|--verbose");
            fc_log_info("    enable debug output");                 
            return 1;
        } else if ( ! strcmp(argv[i],"--config" ) || ! strcmp(argv[i],"-c" ) ) {          
            if ( (i+1) >= argc ) {
                fc_log_err("main(): --config requires an argument");
                return 1;
            }                
            configFileLocation = argv[i+1];
            i++;
        } else  if ( ! strcmp(argv[i],"--daemon" ) || ! strcmp(argv[i],"-f" )) {          
            fork = 1;
        } else if ( ! strcmp(argv[i],"--verbose" ) || ! strcmp(argv[i],"-v" )) {          
            debugMode = 1;
        } else {
            fc_log_err("main(): Unknown commandline option %s",argv[i]);                
            return 1;                
        }
    }
    
    if ( configFileLocation == NULL ) {
        fc_log_err("main(): --config <configuration file> option is missing");
        return 1;        
    }
    
    if ( fork ) {
        fc_log_debug("main(): Becoming a daemon");
        is_daemon = 1;
        daemonize();
        fc_log_debug("main(): Became a daemon");        
    }        
    
    if ( sensors_init(NULL) ) {
        fc_log_err("main(): Failed to initialize sensors library");
        return 1;
    }
            
    fc_config *config = fc_load_configuration(configFileLocation);    
    if ( config == NULL ) {
        fc_log_err("main(): Failed to load configuration");
        return 1;
    }
    fc_debug_print_config(config);
    
    
    if ( ! fc_ser_open(config) ) 
    {
        fc_log_err("main(): Failed to open serial device %s", config->serial_device);
        return 1;
    }
        
    sd_notifyf(0, "MAINPID=%lu",(unsigned long) getpid());    
    
    sd_notify(0,"READY=1");    
    
    /*
     *  MAIN LOOP
     */
        
    // start with max. fan speed as
    // fan controller starts with 100% fan speed
    // and we don't want to slow it down
    // just to increase the speed again
    // if the temperature is too high from the start
    double previousFanSpeed = config->max_fan_speed;    
    while ( 1 ) 
    {
        sd_notify(0,"WATCHDOG=1");        
        thermal_zone *zone=config->thermal_zones;
        
        double finalFanSpeed = config->max_fan_speed;
        int successfulTempReadingCount = 0;
        while ( zone != NULL )             
        {
            // Get all temperature readings for this thermal zone
            temp_reading *readings = fc_read_temps(zone);            
            if ( readings == NULL ) {
                fc_log_err("main(): Failed to read ANY temperatures for zone '%s'",zone->name);         
                zone = zone->next;
                continue;
            }
            fc_log_debug("main(): Got readings for zone '%s'",zone->name);
            for ( temp_reading *t=readings ; t != NULL ; t=t->next ) {
                fc_print_reading(t);
            }
            
            // find the highest temperature reading for this thermal zone
            temp_reading *current=readings;
            double temp;
            if ( current != NULL ) {
                successfulTempReadingCount++;
                temp = current->value;
                current = current->next;
            }
            while ( current != NULL ) {  
                temp = fmax(temp,current->value);
                current = current->next;
            }
            fc_free_temp_readings(readings);
            
            // find lower and upper bound for 
            // fan speed around the highest measured temperature
            double minTemp = 0;
            double maxTemp = 100;
            int minSpeed = config->min_fan_speed;
            int maxSpeed = config->max_fan_speed;
            for ( fan_speed_mapping *mapping = zone->fan_speed_mapping ; mapping != NULL ; mapping=mapping->next) {
                if ( mapping->temperature <= temp && mapping->temperature >= minTemp ) {
                    minTemp = mapping->temperature;
                    minSpeed = mapping->fan_speed;
                }
                if ( mapping->temperature >= temp && mapping->temperature <= maxTemp ) {
                    maxTemp = mapping->temperature;
                    maxSpeed = mapping->fan_speed;
                }
            }  
            
            fc_log_debug("main(): zone %s , max. temp %f , (%f,%d) - (%f,%d)",zone->name,temp, minTemp, minSpeed, maxTemp, maxSpeed);
        
            // linear interpolation
            double fanSpeed = minSpeed + (temp-minTemp) * ( (maxSpeed-minSpeed)/(double) (maxTemp-minTemp) );
            
            // clamp to [min_fan_speed,max_fan_speed]
            fanSpeed = fmin( fmax(fanSpeed, config->min_fan_speed) , config->max_fan_speed );
              
            fc_log_debug("main(): interpolated speed for zone '%s' - %f",zone->name,fanSpeed);
            
            // update final result
            if ( successfulTempReadingCount == 1 ) {
                // first temperature reading we've got
                finalFanSpeed = fanSpeed;
            } else  {
                // retain the max. fan speed from all the zones as the final result
                finalFanSpeed = fmax(finalFanSpeed,fanSpeed);
            }
            zone = zone->next;
        }
        
        fc_log_debug("main(): desired fan speed %f",finalFanSpeed);          
        
        // exponential moving average
        double averagedSpeed = previousFanSpeed + config->smoothing_alpha * ( finalFanSpeed - previousFanSpeed );
                                      
        previousFanSpeed = averagedSpeed;
        
        fc_log_debug("main(): Setting fan speed %f",averagedSpeed);              
        send_fan_speed(averagedSpeed);
        
        msleep(config->loop_delay_millis);
    }        
}
