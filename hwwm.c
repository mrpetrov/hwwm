/*
* hwwm.c
*
* Raspberry Pi home warm water manager which uses 1wire and GPIO.
* Plamen Petrov
*
* hwwm is Plamen's custom home warm water controller, based on the Raspberry Pi (2,3,4).
* Data is gathered and logged every 10 seconds from 5 DS18B20 waterproof sensors,
* 4 relays are controlled via GPIO, and a GPIO pin is read to note current
* power source: grid or battery backed UPS. Commands for a counterpart system are
* sent via setting 4 designated GPIO ports output, acting as a guaranteed comms channel.
* Log data is in CSV format, to be picked up by some sort of data collection/graphing
* tool, like collectd or similar. There is also JSON file more suitable for sending data
* to data collection software like mqqt/emoncms.
* The daemon is controlled via its configuration file, which hwwm can be told to
* re-read and parse while running to change config in flight. This is done by
* sending SIGUSR1 signal to the daemon process. The event is noted in the log file.
* The logfile itself can be "grep"-ed for "ALARM" and "INFO" to catch and notify
* of notable events, recorded by the daemon.
*/

#ifndef PGMVER
#error Need to define PGMVER in order to compile me!
#endif

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <time.h>

#define RUNNING_DIR     "/tmp"
#define LOCK_FILE       "/run/hwwm.pid"
#define LOG_FILE        "/var/log/hwwm.log"
#define DATA_FILE       "/run/shm/hwwm_data.log"
#define TABLE_FILE      "/run/shm/hwwm_current"
#define JSON_FILE	"/run/shm/hwwm_current_json"
#define CFG_TABLE_FILE  "/run/shm/hwwm_cur_cfg"
#define CONFIG_FILE     "/etc/hwwm.cfg"
#define POWER_FILE      "/var/log/hwwm_power"

#define BUFFER_MAX 3
#define DIRECTION_MAX 35
#define VALUE_MAX 50
#define MAXLEN 80

#define IN  0
#define OUT 1

#define LOW  0
#define HIGH 1

/* Maximum difference allowed for data received from sensors between reads, C */
#define MAX_TEMP_DIFF        7

/* Number of all sensors to be used by the system */
#define TOTALSENSORS         5

/* Array of char* holding the paths to temperature DS18B20 sensors */
char* sensor_paths[TOTALSENSORS+1];

/*  var to keep track of read errors, so if a threshold is reached - the
    program can safely shut down everything, send notification and bail out;
    initialised with borderline value to trigger immediately on errors during
    start-up; the program logic tolerates 1 minute of missing sensor data
*/
unsigned short sensor_read_errors[TOTALSENSORS+1] = { 3, 3, 3, 3, 3, 3 };

/* current sensors temperatures - e.g. values from last read */
float sensors[TOTALSENSORS+1] = { 0, -200, -200, -200, -200, -200 };

/* previous sensors temperatures - e.g. values from previous to last read */
float sensors_prv[TOTALSENSORS+1] = { 0, -200, -200, -200, -200, -200 };

/* and sensor name mappings */
#define   Tkotel                sensors[1]
#define   Tkolektor             sensors[2]
#define   TboilerHigh           sensors[3]
#define   TboilerLow            sensors[4]
#define   Tenv                    sensors[5]

#define   TkotelPrev            sensors_prv[1]
#define   TkolektorPrev         sensors_prv[2]
#define   TboilerHighPrev       sensors_prv[3]
#define   TboilerLowPrev        sensors_prv[4]
#define   TenvPrev                sensors_prv[5]

/* HTTB == Hourly Target Temp Base for furnace water; NB 24:00 = 0;
 *  hwwm will get to the real target by substracting the outside temp
 *  from the values defined in this array */
/*                             0    1    2    3    4    5    6    7   8    9   10  11  12  13  14  15  16  17  18  19  20  21  22  23*/
short HTTB[24] = { 38, 36, 36, 36, 36, 38, 40, 40, 40, 40, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 43, 40 };

#define   currentHTTB       HTTB[current_timer_hour]

float furnace_water_target = 22.33;

/* current controls state - e.g. set on last decision making */
short controls[11] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* and control name mappings */
#define   CPump1                controls[1]
#define   CPump2                controls[2]
#define   CValve                controls[3]
#define   CHeater               controls[4]
#define   CPowerByBattery       controls[5]
#define   CPowerByBatteryPrev   controls[6]
#define   CCommsPin1          controls[7]   // pin/bit to signal need for low heat - one AC can be ON
#define   CCommsPin2          controls[8]   // pin/bit to request full heat (both ACs)
#define   CCommsPin3          controls[9]   // pin/bit to ????
#define   CCommsPin4          controls[10] // pin/bit to signal we on battery power

/* controls state cycles - zeroed on change to state */
long ctrlstatecycles[10] = { -1, 150000, 150000, 2200, 2200, 32, 32, 0, 0, -1 };

#define   SCPump1               ctrlstatecycles[1]
#define   SCPump2               ctrlstatecycles[2]
#define   SCValve               ctrlstatecycles[3]
#define   SCHeater              ctrlstatecycles[4]
#define   SCCommsPin1              ctrlstatecycles[5]
#define   SCCommsPin2              ctrlstatecycles[6]
#define   SCCommsPin3              ctrlstatecycles[7]
#define   SCCommsPin4              ctrlstatecycles[8]

float TotalPowerUsed;
float NightlyPowerUsed;

float nightEnergyTemp;

/* hwwm keeps track of total and night tariff watt-hours electrical power used */
/* night tariff is between 23:00 and 06:00 */
/* constants of Watt-hours of electricity used per 10 secs */
#define   HEATERPPC         8.340
#define   PUMP1PPC          0.135
#define   PUMP2PPC          0.021
#define   VALVEPPC          0.006
#define   SELFPPC           0.022
/* my boiler uses 3kW per hour, so this is 0,00834 kWh per 10 seconds */
/* this in Wh per 10 seconds is 8.34 W */
/* pump 1 (furnace) runs at 48 W setting, pump 2 (solar) - 7 W */

/* NightEnergy (NE) start and end hours variables - get recalculated every day */
unsigned short NEstart = 20;
unsigned short NEstop  = 11;

/* Nubmer of cycles (circa 10 seconds each) that the program has run */
unsigned long ProgramRunCycles  = 0;

/* timers - current hour and month vars - used in keeping things up to date */
unsigned short current_timer_hour = 0;
unsigned short current_month = 0;

/* a var to be non-zero if it is winter time - so furnace should not be allowed to go too cold */
unsigned short now_is_winter = 0;

/* array storing the hour at wich to make the solar pump daily run for each month */
unsigned short pump_start_hour_for[13] = { 11, 14, 13, 12, 11, 10, 9, 9, 10, 11, 12, 13, 14 };

struct cfg_struct
{
    char    tkotel_sensor[MAXLEN];
    char    tkolektor_sensor[MAXLEN];
    char    tboilerh_sensor[MAXLEN];
    char    tboilerl_sensor[MAXLEN];
    char    tenv_sensor[MAXLEN];
    char    bat_powered_pin_str[MAXLEN];
    int     bat_powered_pin;
    char    pump1_pin_str[MAXLEN];
    int     pump1_pin;
    char    pump2_pin_str[MAXLEN];
    int     pump2_pin;
    char    valve1_pin_str[MAXLEN];
    int     valve1_pin;
    char    el_heater_pin_str[MAXLEN];
    int     el_heater_pin;
    char    commspin1_pin_str[MAXLEN];
    int     commspin1_pin;
    char    commspin2_pin_str[MAXLEN];
    int     commspin2_pin;
    char    commspin3_pin_str[MAXLEN];
    int     commspin3_pin;
    char    commspin4_pin_str[MAXLEN];
    int     commspin4_pin;
    char    invert_output_str[MAXLEN];
    int     invert_output;
    char    mode_str[MAXLEN];
    int     mode;
    char    wanted_T_str[MAXLEN];
    int     wanted_T;
    char    use_electric_heater_night_str[MAXLEN];
    int     use_electric_heater_night;
    char    use_electric_heater_day_str[MAXLEN];
    int     use_electric_heater_day;
    char    pump1_always_on_str[MAXLEN];
    int     pump1_always_on;
    char    use_pump1_str[MAXLEN];
    int     use_pump1;
    char    use_pump2_str[MAXLEN];
    int     use_pump2;
    char    day_to_reset_Pcounters_str[MAXLEN];
    int     day_to_reset_Pcounters;
    char    night_boost_str[MAXLEN];
    int     night_boost;
    char    abs_max_str[MAXLEN];
    int     abs_max;
    char    max_big_consumers_str[MAXLEN];
    int     max_big_consumers;
}
cfg_struct;

struct cfg_struct cfg;

short need_to_read_cfg = 0;

short just_started = 0;

/* FORWARD DECLARATIONS so functions can be used in preceding ones */
short
DisableGPIOpins();
/* end of forward-declared functions */

void
rangecheck_GPIO_pin( int p )
{
    if (p < 4) p = 4;
    if (p > 27) p = 27;
}

short
not_every_GPIO_pin_is_UNIQUE()
{
	short result=0;
	if (cfg.bat_powered_pin == cfg.pump1_pin) result++;
	if (cfg.bat_powered_pin == cfg.pump2_pin) result++;
	if (cfg.bat_powered_pin == cfg.valve1_pin) result++;
	if (cfg.bat_powered_pin == cfg.el_heater_pin) result++;
	if (cfg.bat_powered_pin == cfg.commspin1_pin) result++;
	if (cfg.bat_powered_pin == cfg.commspin2_pin) result++;
	if (cfg.bat_powered_pin == cfg.commspin3_pin) result++;
	if (cfg.bat_powered_pin == cfg.commspin4_pin) result++;
	if (cfg.pump1_pin == cfg.pump2_pin) result++;
	if (cfg.pump1_pin == cfg.valve1_pin) result++;
	if (cfg.pump1_pin == cfg.el_heater_pin) result++;
	if (cfg.pump1_pin == cfg.commspin1_pin) result++;
	if (cfg.pump1_pin == cfg.commspin2_pin) result++;
	if (cfg.pump1_pin == cfg.commspin3_pin) result++;
	if (cfg.pump1_pin == cfg.commspin4_pin) result++;
	if (cfg.pump2_pin == cfg.valve1_pin) result++;
	if (cfg.pump2_pin == cfg.el_heater_pin) result++;
	if (cfg.pump2_pin == cfg.commspin1_pin) result++;
	if (cfg.pump2_pin == cfg.commspin2_pin) result++;
	if (cfg.pump2_pin == cfg.commspin3_pin) result++;
	if (cfg.pump2_pin == cfg.commspin4_pin) result++;
	if (cfg.valve1_pin == cfg.el_heater_pin) result++;
	if (cfg.valve1_pin == cfg.commspin1_pin) result++;
	if (cfg.valve1_pin == cfg.commspin2_pin) result++;
	if (cfg.valve1_pin == cfg.commspin3_pin) result++;
	if (cfg.valve1_pin == cfg.commspin4_pin) result++;
	if (cfg.commspin1_pin == cfg.commspin2_pin) result++;
	if (cfg.commspin1_pin == cfg.commspin3_pin) result++;
	if (cfg.commspin1_pin == cfg.commspin4_pin) result++;
	if (cfg.commspin2_pin == cfg.commspin3_pin) result++;
	if (cfg.commspin2_pin == cfg.commspin4_pin) result++;
	if (cfg.commspin3_pin == cfg.commspin4_pin) result++;
	return result;
}

void
rangecheck_mode( int m )
{
    if (m < 0) m = 0;
    if (m > 8) m = 0;
}

void
rangecheck_wanted_temp( int temp )
{
    if (temp < 25) temp = 25;
    if (temp > 62) temp = 62;
}

void
rangecheck_abs_max_temp( int t )
{
    if (t < 40) t = 40;
    if (t > 70) t = 70;
    if (t < (cfg.wanted_T+3)) { t = cfg.wanted_T+3; }
}

void
rangecheck_max_big_consumers( int t )
{
    if (t < 1) t = 1;
    if (t > 3) t = 3;
}

void
rangecheck_day_of_month( int d )
{
    if (d < 1) d = 1;
    if (d > 28) d = 28;
}

void
SetDefaultPINs() {
    cfg.bat_powered_pin = 7;
    cfg.pump1_pin = 5;
    cfg.pump2_pin = 6;
    cfg.valve1_pin = 13;
    cfg.el_heater_pin = 16;
    cfg.commspin1_pin = 17;
    cfg.commspin2_pin = 18;
    cfg.commspin3_pin = 27;
    cfg.commspin4_pin = 22;
    cfg.max_big_consumers = 1;
}

void
SetDefaultCfg() {
    strcpy( cfg.tkotel_sensor, "/dev/zero/1");
    strcpy( cfg.tkolektor_sensor, "/dev/zero/2");
    strcpy( cfg.tboilerh_sensor, "/dev/zero/3");
    strcpy( cfg.tboilerl_sensor, "/dev/zero/4");
    strcpy( cfg.tenv_sensor, "/dev/zero/5");
    SetDefaultPINs();
    cfg.invert_output = 1;
    cfg.mode = 1;
    cfg.wanted_T = 40;
    cfg.use_electric_heater_night = 1;
    cfg.use_electric_heater_day = 1;
    cfg.pump1_always_on = 0;
    cfg.use_pump1 = 1;
    cfg.use_pump2 = 1;
    cfg.day_to_reset_Pcounters = 4;
    cfg.night_boost = 0;
    cfg.abs_max = 63;

    nightEnergyTemp = 0;
    sensor_paths[0] = (char *) &cfg.tkotel_sensor;
    sensor_paths[1] = (char *) &cfg.tkotel_sensor;
    sensor_paths[2] = (char *) &cfg.tkolektor_sensor;
    sensor_paths[3] = (char *) &cfg.tboilerh_sensor;
    sensor_paths[4] = (char *) &cfg.tboilerl_sensor;
    sensor_paths[5] = (char *) &cfg.tenv_sensor;
}

short
log_message(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );
    sprintf( file_string, "%s %s", timestamp, message );
    logfile = fopen( filename, "a" );
    if ( !logfile ) return -1;
    fprintf( logfile, "%s\n", file_string );
    fclose( logfile );
    return 0;
}

/* this version of the logging function destroys the opened file contents */
void
log_msg_ovr(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );
    sprintf( file_string, "%s%s", timestamp, message );
    logfile = fopen( filename, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "%s\n", file_string );
    fclose( logfile );
}

/* this version of the logging function destroys the opened file contents, no timestamp and new line */
void
log_msg_cln(char *filename, char *message) {
    FILE *logfile;
    char file_string[300];

    sprintf( file_string, "%s", message );
    logfile = fopen( filename, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "%s", file_string );
    fclose( logfile );
}

/* trim: get rid of trailing and leading whitespace...
    ...including the annoying "\n" from fgets()
*/
char *
trim (char * s)
{
    /* Initialize start, end pointers */
    char *s1 = s, *s2 = &s[strlen (s) - 1];

    /* Trim and delimit right side */
    while ( (isspace (*s2)) && (s2 >= s1) )
    s2--;
    *(s2+1) = '\0';

    /* Trim left side */
    while ( (isspace (*s1)) && (s1 < s2) )
    s1++;

    /* Copy finished string */
    strcpy (s, s1);
    return s;
}

void
parse_config()
{
    int i = 0;
    char *s, buff[150];
    FILE *fp = fopen(CONFIG_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE,"WARNING: Failed to open "CONFIG_FILE" file for reading!");
        } else {
        /* Read next line */
        while ((s = fgets (buff, sizeof buff, fp)) != NULL)
        {
            /* Skip blank lines and comments */
            if (buff[0] == '\n' || buff[0] == '#')
            continue;

            /* Parse name/value pair from line */
            char name[MAXLEN], value[MAXLEN];
            s = strtok (buff, "=");
            if (s==NULL) continue;
            else strncpy (name, s, MAXLEN);
            s = strtok (NULL, "=");
            if (s==NULL) continue;
            else strncpy (value, s, MAXLEN);
            trim (value);

            /* Copy into correct entry in parameters struct */
            if (strcmp(name, "tkotel_sensor")==0)
            strncpy (cfg.tkotel_sensor, value, MAXLEN);
            else if (strcmp(name, "tkolektor_sensor")==0)
            strncpy (cfg.tkolektor_sensor, value, MAXLEN);
            else if (strcmp(name, "tboilerh_sensor")==0)
            strncpy (cfg.tboilerh_sensor, value, MAXLEN);
            else if (strcmp(name, "tboilerl_sensor")==0)
            strncpy (cfg.tboilerl_sensor, value, MAXLEN);
            else if (strcmp(name, "tenv_sensor")==0)
            strncpy (cfg.tenv_sensor, value, MAXLEN);
            else if (strcmp(name, "bat_powered_pin")==0)
            strncpy (cfg.bat_powered_pin_str, value, MAXLEN);
            else if (strcmp(name, "pump1_pin")==0)
            strncpy (cfg.pump1_pin_str, value, MAXLEN);
            else if (strcmp(name, "pump2_pin")==0)
            strncpy (cfg.pump2_pin_str, value, MAXLEN);
            else if (strcmp(name, "valve1_pin")==0)
            strncpy (cfg.valve1_pin_str, value, MAXLEN);
            else if (strcmp(name, "el_heater_pin")==0)
            strncpy (cfg.el_heater_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin1_pin")==0)
            strncpy (cfg.commspin1_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin2_pin")==0)
            strncpy (cfg.commspin2_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin3_pin")==0)
            strncpy (cfg.commspin3_pin_str, value, MAXLEN);
            else if (strcmp(name, "commspin4_pin")==0)
            strncpy (cfg.commspin4_pin_str, value, MAXLEN);
            else if (strcmp(name, "invert_output")==0)
            strncpy (cfg.invert_output_str, value, MAXLEN);
            else if (strcmp(name, "mode")==0)
            strncpy (cfg.mode_str, value, MAXLEN);
            else if (strcmp(name, "wanted_T")==0)
            strncpy (cfg.wanted_T_str, value, MAXLEN);
            else if (strcmp(name, "use_electric_heater_night")==0)
            strncpy (cfg.use_electric_heater_night_str, value, MAXLEN);
            else if (strcmp(name, "use_electric_heater_day")==0)
            strncpy (cfg.use_electric_heater_day_str, value, MAXLEN);
            else if (strcmp(name, "pump1_always_on")==0)
            strncpy (cfg.pump1_always_on_str, value, MAXLEN);
            else if (strcmp(name, "use_pump1")==0)
            strncpy (cfg.use_pump1_str, value, MAXLEN);
            else if (strcmp(name, "use_pump2")==0)
            strncpy (cfg.use_pump2_str, value, MAXLEN);
            else if (strcmp(name, "day_to_reset_Pcounters")==0)
            strncpy (cfg.day_to_reset_Pcounters_str, value, MAXLEN);
            else if (strcmp(name, "night_boost")==0)
            strncpy (cfg.night_boost_str, value, MAXLEN);
            else if (strcmp(name, "abs_max")==0)
            strncpy (cfg.abs_max_str, value, MAXLEN);
            else if (strcmp(name, "max_big_consumers")==0)
            strncpy (cfg.max_big_consumers_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    /* Convert strings to int */
    strcpy( buff, cfg.bat_powered_pin_str );
    i = atoi( buff );
    cfg.bat_powered_pin = i;
    rangecheck_GPIO_pin( cfg.bat_powered_pin );
    strcpy( buff, cfg.pump1_pin_str );
    i = atoi( buff );
    cfg.pump1_pin = i;
    rangecheck_GPIO_pin( cfg.pump1_pin );
    strcpy( buff, cfg.pump2_pin_str );
    i = atoi( buff );
    cfg.pump2_pin = i;
    rangecheck_GPIO_pin( cfg.pump2_pin );
    strcpy( buff, cfg.valve1_pin_str );
    i = atoi( buff );
    cfg.valve1_pin = i;
    rangecheck_GPIO_pin( cfg.valve1_pin );
    strcpy( buff, cfg.el_heater_pin_str );
    i = atoi( buff );
    cfg.el_heater_pin = i;
    rangecheck_GPIO_pin( cfg.el_heater_pin );
    strcpy( buff, cfg.commspin1_pin_str );
    i = atoi( buff );
    cfg.commspin1_pin = i;
    rangecheck_GPIO_pin( cfg.commspin1_pin );
    strcpy( buff, cfg.commspin2_pin_str );
    i = atoi( buff );
    cfg.commspin2_pin = i;
    rangecheck_GPIO_pin( cfg.commspin2_pin );
    strcpy( buff, cfg.commspin3_pin_str );
    i = atoi( buff );
    cfg.commspin3_pin = i;
    rangecheck_GPIO_pin( cfg.commspin3_pin );
    strcpy( buff, cfg.commspin4_pin_str );
    i = atoi( buff );
    cfg.commspin4_pin = i;
    rangecheck_GPIO_pin( cfg.commspin4_pin );
	if (not_every_GPIO_pin_is_UNIQUE()) {
       log_message(LOG_FILE,"ALERT: Check config - found configured GPIO pin assigned more than once!");
       log_message(LOG_FILE,"ALERT: The above is an error. Switching to using default GPIO pins config...");
       SetDefaultPINs();
	}
    strcpy( buff, cfg.invert_output_str );
    i = atoi( buff );
    cfg.invert_output = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */

    strcpy( buff, cfg.mode_str );
    i = atoi( buff );
    cfg.mode = i;
    rangecheck_mode( cfg.mode );
    strcpy( buff, cfg.wanted_T_str );
    i = atoi( buff );
    cfg.wanted_T = i;
    rangecheck_wanted_temp( cfg.wanted_T );
    strcpy( buff, cfg.use_electric_heater_night_str );
    i = atoi( buff );
    cfg.use_electric_heater_night = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_electric_heater_day_str );
    i = atoi( buff );
    cfg.use_electric_heater_day = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.pump1_always_on_str );
    i = atoi( buff );
    cfg.pump1_always_on = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_pump1_str );
    i = atoi( buff );
    cfg.use_pump1 = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.use_pump2_str );
    i = atoi( buff );
    cfg.use_pump2 = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.day_to_reset_Pcounters_str );
    i = atoi( buff );
    cfg.day_to_reset_Pcounters = i;
    rangecheck_day_of_month( cfg.day_to_reset_Pcounters );
    strcpy( buff, cfg.night_boost_str );
    i = atoi( buff );
    cfg.night_boost = i;
    /* ^ no need for range check - 0 is OFF, non-zero is ON */
    strcpy( buff, cfg.abs_max_str );
    i = atoi( buff );
    cfg.abs_max = i;
    rangecheck_abs_max_temp( cfg.abs_max );


    strcpy( buff, cfg.max_big_consumers_str );
    i = atoi( buff );
    cfg.max_big_consumers = i;
    rangecheck_max_big_consumers( cfg.max_big_consumers );


    /* Prepare log messages with sensor paths and write them to log file */
    sprintf( buff, "Furnace temp sensor file: %s", cfg.tkotel_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Solar collector temp sensor file: %s", cfg.tkolektor_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Boiler high temp sensor file: %s", cfg.tboilerh_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Boiler low temp sensor file: %s", cfg.tboilerl_sensor );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Outdoor environment temp sensor file: %s", cfg.tenv_sensor );
    log_message(LOG_FILE, buff);
    /* Prepare log messages with GPIO pins used and write them to log file */
    sprintf( buff, "Using INPUT GPIO pins (BCM mode) as follows: battery powered: %d", cfg.bat_powered_pin );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Using COMMs GPIO pins (BCM mode) as follows: comms1: %d, comms2: %d, comms3: %d, "\
	"comms4: %d ", cfg.commspin1_pin, cfg.commspin2_pin, cfg.commspin3_pin, cfg.commspin4_pin );
    log_message(LOG_FILE, buff);
    sprintf( buff, "Using OUTPUT GPIO pins (BCM mode) as follows: P1: %d, P2: %d, V: %d, "\
	"H: %d ", cfg.pump1_pin, cfg.pump2_pin, cfg.valve1_pin, cfg.el_heater_pin );
    log_message(LOG_FILE, buff);
    if (cfg.invert_output) {
        sprintf( buff, "OUTPUT GPIO pins controlling is INVERTED - ON is LOW (0)" );
        log_message(LOG_FILE, buff);
    }
    else {
        sprintf( buff, "OUTPUT GPIO pins controlling is STRAIGHT - ON is HIGH (1)" );
        log_message(LOG_FILE, buff);
    }
    /* Prepare log message part 1 and write it to log file */
    if (fp == NULL) {
        sprintf( buff, "INFO: Using values: Mode=%d, wanted temp=%d, el. heater: night=%d, day=%d,",\
        cfg.mode, cfg.wanted_T, cfg.use_electric_heater_night, cfg.use_electric_heater_day );
        } else {
        sprintf( buff, "INFO: Read CFG file: Mode=%d, wanted temp=%d, el. heater: night=%d, day=%d,",\
        cfg.mode, cfg.wanted_T, cfg.use_electric_heater_night, cfg.use_electric_heater_day );
    }
    log_message(LOG_FILE, buff);
    /* Prepare log message part 2 and write it to log file */
    sprintf( buff, "INFO: Furnace pump always on=%d, use furnace pump=%d, use solar pump=%d, reset P counters day=%d", 
                cfg.pump1_always_on, cfg.use_pump1, cfg.use_pump2, cfg.day_to_reset_Pcounters);
    log_message(LOG_FILE, buff);
    sprintf( buff, "INFO: Night boiler boost=%d, absMAX=%d, max big consumers=%d", 
                cfg.night_boost, cfg.abs_max, cfg.max_big_consumers );
    log_message(LOG_FILE, buff);
	
    /* stuff for after parsing config file: */
    /* calculate maximum possible temp for use in night_boost case */
    nightEnergyTemp = ((float)cfg.wanted_T + 12);
    if (nightEnergyTemp > (float)cfg.abs_max) { nightEnergyTemp = (float)cfg.abs_max; }
}

void
WritePersistentPower() {
    FILE *logfile;
    char timestamp[30];
    time_t t;
    struct tm *t_struct;

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( timestamp, sizeof timestamp, "%F %T", t_struct );

    logfile = fopen( POWER_FILE, "w" );
    if ( !logfile ) return;
    fprintf( logfile, "# hwwm power persistence file written %s\n", timestamp );
    fprintf( logfile, "total=%6.3f\n", TotalPowerUsed );
    fprintf( logfile, "nightly=%6.3f\n", NightlyPowerUsed );
    fclose( logfile );
}

void
ReadPersistentPower() {
    float f = 0;
    char *s, buff[150];
    char totalP_str[MAXLEN];
    char nightlyP_str[MAXLEN];
    short should_write=0;
    strcpy( totalP_str, "0" );
    strcpy( nightlyP_str, "0" );
    FILE *fp = fopen(POWER_FILE, "r");
    if (fp == NULL) {
        log_message(LOG_FILE,"WARNING: Failed to open "POWER_FILE" file for reading!");
        should_write = 1;
        } else {
        /* Read next line */
        while ((s = fgets (buff, sizeof buff, fp)) != NULL)
        {
            /* Skip blank lines and comments */
            if (buff[0] == '\n' || buff[0] == '#')
            continue;

            /* Parse name/value pair from line */
            char name[MAXLEN], value[MAXLEN];
            s = strtok (buff, "=");
            if (s==NULL) continue;
            else strncpy (name, s, MAXLEN);
            s = strtok (NULL, "=");
            if (s==NULL) continue;
            else strncpy (value, s, MAXLEN);
            trim (value);

            /* Copy data in corresponding strings */
            if (strcmp(name, "total")==0)
            strncpy (totalP_str, value, MAXLEN);
            else if (strcmp(name, "nightly")==0)
            strncpy (nightlyP_str, value, MAXLEN);
        }
        /* Close file */
        fclose (fp);
    }

    if (should_write) {
        log_message(LOG_FILE, "Creating missing power persistence data file...");
        WritePersistentPower();
    }
    else {
        /* Convert strings to float */
        strcpy( buff, totalP_str );
        f = atof( buff );
        TotalPowerUsed = f;
        strcpy( buff, nightlyP_str );
        f = atof( buff );
        NightlyPowerUsed = f;
    }

    /* Prepare log message and write it to log file */
    if (fp == NULL) {
        sprintf( buff, "INFO: Using power counters start values: Total=%6.3f, Nightly=%6.3f",
        TotalPowerUsed, NightlyPowerUsed );
        } else {
        sprintf( buff, "INFO: Read power counters start values: Total=%6.3f, Nightly=%6.3f",
        TotalPowerUsed, NightlyPowerUsed );
    }
    log_message(LOG_FILE, buff);
}

int
GPIOExport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO export for writing!");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

int
GPIOUnexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_written;
    int fd;

    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO unexport for writing!");
        return(-1);
    }

    bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
    write(fd, buffer, bytes_written);
    close(fd);
    return(0);
}

int
GPIODirection(int pin, int dir)
{
    static const char s_directions_str[]  = "in\0out";

    char path[DIRECTION_MAX];
    int fd;

    snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO direction for writing!");
        return(-1);
    }

    if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3], IN == dir ? 2 : 3)) {
        log_message(LOG_FILE,"Failed to set GPIO direction!");
        return(-1);
    }

    close(fd);
    return(0);
}

int
GPIORead(int pin)
{
    char path[VALUE_MAX];
    char value_str[3];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO value for reading!");
        return(-1);
    }

    if (-1 == read(fd, value_str, 3)) {
        log_message(LOG_FILE,"Failed to read GPIO value!");
        return(-1);
    }

    close(fd);

    return(atoi(value_str));
}

int
GPIOWrite(int pin, int value)
{
    static const char s_values_str[] = "01";

    char path[VALUE_MAX];
    int fd;

    snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Failed to open GPIO value for writing!");
        return(-1);
    }

    if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
        log_message(LOG_FILE,"Failed to write GPIO value!");
        return(-1);
    }

    close(fd);
    return(0);
}

/*
    Example output of a sensor file:

    pi@raspberrypi ~ $ time cat /sys/bus/w1/devices/28-041464764cff/w1_slave
    84 01 55 00 3f ff 3f 10 d7 : crc=d7 YES
    84 01 55 00 3f ff 3f 10 d7 t=24250

    real    0m0.834s
    user    0m0.000s
    sys     0m0.050s
*/

float
sensorRead(const char* sensor)
{
    char path[VALUE_MAX];
    char value_str[50];
    int fd;
    char *str = "84 01 55 00 3f ff 3f 10 d7 t=114250";
    const char *result = str;
    long int_temp = 0;
    float temp = -200;
    /* if having trouble - return -200 */

    /* try to open sensor file */
    snprintf(path, VALUE_MAX, "%s", sensor);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        log_message(LOG_FILE,"Error opening sensor file. Continuing.");
        return(temp);
    }

    /* read the first line of data */
    if (-1 == read(fd, value_str, 39)) {
        log_message(LOG_FILE,"Error reading from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* throw the first line away */
    strncpy(value_str, " ", 48);

    /* read the second line into value_str */
    if (-1 == read(fd, value_str, 35)) {
        log_message(LOG_FILE,"Error reading row 2 from sensor file. Continuing.");
        close(fd);
        return(temp);
    }

    /* close the file - we are done with it */
    close(fd);

    /* transform sensor data to float */
    if((result = strchr((char *)&value_str, '=')) != NULL) {
        /* increment result to avoid the '=' */
        ++result;
        int_temp = atol( result );
        temp = ((float)int_temp) / 1000;
    }

    /* return the read temperature */
    return(temp);
}

void
signal_handler(int sig)
{
    switch(sig) {
        case SIGUSR1:
        log_message(LOG_FILE, "INFO: Signal SIGUSR1 caught. Will re-read config file soon.");
        need_to_read_cfg = 1;
        break;
        case SIGUSR2:
        log_message(LOG_FILE, "INFO: Signal SIGUSR2 caught. Not implemented. Continuing.");
        break;
        case SIGHUP:
        log_message(LOG_FILE, "INFO: Signal SIGHUP caught. Not implemented. Continuing.");
        break;
        case SIGTERM:
        log_message(LOG_FILE, "INFO: Terminate signal caught. Stopping.");
        WritePersistentPower();
        if ( ! DisableGPIOpins() ) {
            log_message(LOG_FILE, "WARNING: Errors disabling GPIO pins! Quitting anyway.");
            exit(14);
        }
        // this run was ProgramRunCycles cycles ;) 
        log_message(LOG_FILE,"Exiting normally. Bye, bye!");
        exit(0);
        break;
    }
}

void
daemonize()
{
    int i, lfp;
    char str[10];

    if(getppid()==1) return; /* already a daemon */
    i=fork();
    if (i<0) { printf("hwwm daemonize(): Fork error!\n"); exit(1); }/* fork error */
    if (i>0) exit(0); /* parent exits */
    /* child (daemon) continues */
    setsid(); /* obtain a new process group */
    for (i=getdtablesize();i>=0;--i) close(i); /* close all descriptors */
    i=open("/dev/null",O_RDWR); dup(i); dup(i); /* handle standart I/O */
    umask(022); /* set newly created file permissions */
    chdir(RUNNING_DIR); /* change running directory */
    lfp=open(LOCK_FILE,O_RDWR|O_CREAT,0644);
    if (lfp<0) exit(2); /* can not open */
    if (lockf(lfp,F_TLOCK,0)<0) exit(0); /* can not lock */
    /* first instance continues */
    sprintf(str,"%d\n",getpid());
    write(lfp,str,strlen(str)); /* record pid to lockfile */
    signal(SIGCHLD,SIG_IGN); /* ignore child */
    signal(SIGTSTP,SIG_IGN); /* ignore tty signals */
    signal(SIGTTOU,SIG_IGN);
    signal(SIGTTIN,SIG_IGN);
    signal(SIGUSR1,signal_handler); /* catch signal USR1 */
    signal(SIGUSR2,signal_handler); /* catch signal USR2 */
    signal(SIGHUP,signal_handler); /* catch hangup signal */
    signal(SIGTERM,signal_handler); /* catch kill signal */
}

/* the following 3 functions RETURN 0 ON ERROR! (its to make the program nice to read) */
short
EnableGPIOpins()
{
    if (-1 == GPIOExport(cfg.pump1_pin)) return 0;
    if (-1 == GPIOExport(cfg.pump2_pin)) return 0;
    if (-1 == GPIOExport(cfg.valve1_pin)) return 0;
    if (-1 == GPIOExport(cfg.el_heater_pin))  return 0;
    if (-1 == GPIOExport(cfg.bat_powered_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin1_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin2_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin3_pin)) return 0;
    if (-1 == GPIOExport(cfg.commspin4_pin)) return 0;
    return -1;
}

short
SetGPIODirection()
{
    /* input pins */
    if (-1 == GPIODirection(cfg.bat_powered_pin, IN))  return 0;
    /* output pins */
    if (-1 == GPIODirection(cfg.pump1_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.pump2_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.valve1_pin, OUT)) return 0;
    if (-1 == GPIODirection(cfg.el_heater_pin, OUT))  return 0;
    if (-1 == GPIODirection(cfg.commspin1_pin, OUT))  return 0;
    if (-1 == GPIODirection(cfg.commspin2_pin, OUT))  return 0;
    if (-1 == GPIODirection(cfg.commspin3_pin, OUT))  return 0;
    if (-1 == GPIODirection(cfg.commspin4_pin, OUT))  return 0;
    return -1;
}

short
DisableGPIOpins()
{
    if (-1 == GPIOUnexport(cfg.pump1_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.pump2_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.valve1_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.el_heater_pin))  return 0;
    if (-1 == GPIOUnexport(cfg.bat_powered_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin1_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin2_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin3_pin)) return 0;
    if (-1 == GPIOUnexport(cfg.commspin4_pin)) return 0;
    return -1;
}

void
ReadSensors() {
    float new_val = 0;
    int i;
    char msg[100];

    for (i=1;i<=TOTALSENSORS;i++) {
        new_val = sensorRead(sensor_paths[i]);
        if ( new_val != -200 ) {
            if (sensor_read_errors[i]) sensor_read_errors[i]--;
            if (just_started) { sensors_prv[i] = new_val; sensors[i] = new_val; }
            if (new_val < (sensors_prv[i]-(2*MAX_TEMP_DIFF))) {
                sprintf( msg, "WARNING: Counting %6.3f for sensor %d as BAD and using %6.3f.", new_val, i, sensors_prv[i] );
                log_message(LOG_FILE, msg);
                new_val = sensors_prv[i];
                sensor_read_errors[i]++;
            }
            if (new_val > (sensors_prv[i]+(2*MAX_TEMP_DIFF))) {
                sprintf( msg, "WARNING: Counting %6.3f for sensor %d as BAD and using %6.3f.", new_val, i, sensors_prv[i] );
                log_message(LOG_FILE, msg);
                new_val = sensors_prv[i];
                sensor_read_errors[i]++;
            }
            if (new_val < (sensors_prv[i]-MAX_TEMP_DIFF)) {
                sprintf( msg, "WARNING: Correcting LOW %6.3f for sensor %d with %6.3f.", new_val, i, sensors_prv[i]-MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors_prv[i]-MAX_TEMP_DIFF;
            }
            if (new_val > (sensors_prv[i]+MAX_TEMP_DIFF)) {
                sprintf( msg, "WARNING: Correcting HIGH %6.3f for sensor %d with %6.3f.", new_val, i, sensors_prv[i]+MAX_TEMP_DIFF );
                log_message(LOG_FILE, msg);
                new_val = sensors_prv[i]+MAX_TEMP_DIFF;
            }
            sensors_prv[i] = sensors[i];
            sensors[i] = new_val;
        }
        else {
            sensor_read_errors[i]++;
            sprintf( msg, "WARNING: Sensor %d ReadSensors() errors++. Counter at %d.", i, sensor_read_errors[i] );
            log_message(LOG_FILE, msg);
        }
    }
    /* Allow for maximum of 6 consecutive 10 second intervals of missing sensor data
    on any of the sensors before quitting screaming... */
    for (i=1;i<=TOTALSENSORS;i++) {
        if (sensor_read_errors[i]>5) {
            /* log the errors, clean up and bail out */
            log_message(LOG_FILE, "ALARM: Too many sensor read errors! Stopping.");
            if ( ! DisableGPIOpins() ) {
                log_message(LOG_FILE, "ALARM: GPIO disable failed on handling sensor read failures.");
                exit(66);
            }
            exit(55);
        }
    }
}

/* Read cfg.bat_powered_pin into CPowerByBattery which should be 
set to 1 if the external power is generated by UPS
This also keeps a copy of this info in CCommsPin4 */
void
ReadExternalPower() {
    CPowerByBatteryPrev = CPowerByBattery;
    CPowerByBattery = GPIORead(cfg.bat_powered_pin);
}

/* Function to make GPIO state represent what is in controls[] */
void
ControlStateToGPIO() {
    /* put state on GPIO pins */
    GPIOWrite( cfg.commspin1_pin,  CCommsPin1 );
    GPIOWrite( cfg.commspin2_pin,  CCommsPin2 );
    GPIOWrite( cfg.commspin3_pin,  CCommsPin3 );
    GPIOWrite( cfg.commspin4_pin,  CCommsPin4 );
    if (cfg.invert_output) {
            GPIOWrite( cfg.pump1_pin, !CPump1 );
            GPIOWrite( cfg.pump2_pin, !CPump2 );
            GPIOWrite( cfg.valve1_pin, !CValve );
            GPIOWrite( cfg.el_heater_pin,  !CHeater );
    }
    else {
            GPIOWrite( cfg.pump1_pin, CPump1 );
            GPIOWrite( cfg.pump2_pin, CPump2 );
            GPIOWrite( cfg.valve1_pin, CValve );
            GPIOWrite( cfg.el_heater_pin,  CHeater );
    }
}

void
write_log_start() {
    char start_log_text[80];

    log_message(LOG_FILE,"INFO: hwwm "PGMVER" now starting up...");
    log_message(LOG_FILE,"Running in "RUNNING_DIR", config file "CONFIG_FILE );
    log_message(LOG_FILE,"PID written to "LOCK_FILE", writing CSV data to "DATA_FILE );
    log_message(LOG_FILE,"Writing table data for collectd to "TABLE_FILE );
    log_message(LOG_FILE,"Power used persistence file "POWER_FILE );
    sprintf( start_log_text, "Powers: heater=%3.1f W, pump1=%3.1f W, pump2=%3.1f W",
    HEATERPPC*(6*60), PUMP1PPC*(6*60), PUMP2PPC*(6*60) );
    log_message(LOG_FILE, start_log_text );
    sprintf( start_log_text, "Powers: valve=%3.1f W, self=%3.1f W",
    VALVEPPC*(6*60), SELFPPC*(6*60) );
    log_message(LOG_FILE, start_log_text );
}

/* Function to log currently used config in TABLE_FILE format. The idea is that this file will be made
available to a web app, which will fetch it once in a while to get current working config for hwwm
without the need for root access (necessary to read /etc/hwwm.cfg), so relevant data could be shown.
This function should be called less often, e.g. once every 5 minutes or something... */
void
ReWrite_CFG_TABLE_FILE() {
    static char data[280];
    /* Log data like so:
    Time(by log function),mode,wanted_T,use_electric_heater_night,use_electric_heater_day,
	pump1_always_on,use_pump1,use_pump2,day_to_reset_Pcounters,night_boost,abs_max;
	on seperate lines */
    sprintf( data, ",mode,%d\n_,Tboiler_wanted,%d\n_,elh_nt,%d\n_,elh_dt,%d\n"\
    "_,p1_always_on,%d\n_,use_p1,%d\n_,use_p2,%d\n_,Pcounters_rst_day,%d\n"\
	"_,use_night_boost,%d\n_,Tboiler_absMax,%d",
    cfg.mode, cfg.wanted_T, cfg.use_electric_heater_night, cfg.use_electric_heater_day,
	cfg.pump1_always_on, cfg.use_pump1, cfg.use_pump2, cfg.day_to_reset_Pcounters,
	cfg.night_boost, cfg.abs_max);
    log_msg_ovr(CFG_TABLE_FILE, data);
}

/* Function to get current time and put the hour in current_timer_hour */
void
GetCurrentTime() {
    static char buff[80];
    time_t t;
    struct tm *t_struct;
    short adjusted = 0;
    short must_check = 0;
    unsigned short current_day_of_month = 0;
	
	ReWrite_CFG_TABLE_FILE();

    t = time(NULL);
    t_struct = localtime( &t );
    strftime( buff, sizeof buff, "%H", t_struct );

    current_timer_hour = atoi( buff );
    
    /* do furnace water target temp adjusment based on the environment temp, if it looks reasonable */
    if ( (Tenv > -30) && (Tenv < 50) ) { furnace_water_target = currentHTTB - Tenv; }

    if ((current_timer_hour == 8) && ((ProgramRunCycles % (6*60)) == 0)) must_check = 1;

    /* adjust night tariff start and stop hours at program start and
    every day sometime between 8:00 and 9:00 */
    if (( just_started ) || ( must_check )) {
        strftime( buff, sizeof buff, "%m", t_struct );
        current_month = atoi( buff );
        if ((current_month >= 4)&&(current_month <= 10)) {
            /* April through October - use NE from 23:00 till 6:59 */
            if (NEstart != 23) {
                adjusted = 1;
                NEstart = 23;
                NEstop  = 6;
            }
            now_is_winter = 0;
        }
        else {
            /* November through March - use NE from 22:00 till 5:59 */
            if (NEstart != 22) {
                adjusted = 1;
                NEstart = 22;
                NEstop  = 5;
            }
            now_is_winter = 1;
        }
        if (adjusted) {
            sprintf( buff, "INFO: Adjusted night energy hours, start %.2hu:00,"\
            " stop %.2hu:59.", NEstart, NEstop );
            log_message(LOG_FILE, buff);
        }
        /* among other things - manage power used counters; only check one
        time during the day: at 8'something...*/
        if (must_check) {
            strftime( buff, sizeof buff, "%e", t_struct );
            current_day_of_month = atoi( buff );
            if (current_day_of_month == cfg.day_to_reset_Pcounters) {
                /*...if it is the correct day of month - log gathered data and reset counters */
                sprintf( buff, "INFO: Power used last month: nightly: %3.1f Wh, daily: %3.1f Wh;",
                NightlyPowerUsed, (TotalPowerUsed-NightlyPowerUsed) );
                log_message(LOG_FILE, buff);
                sprintf( buff, "INFO: Total: %3.1f Wh. Power counters reset.", TotalPowerUsed );
                log_message(LOG_FILE, buff);
                TotalPowerUsed = 0;
                NightlyPowerUsed = 0;
            }
        }
    }
}

void
LogData(short HM) {
    static char data[280];
    /* Log data like so:
        Time(by log function) HOUR, TKOTEL,TSOLAR,TBOILERL,TBOILERH,TENV, BOILERTEMPWANTED,BOILERABSMAX,
    furnace_water_target,NIGHTBOOST, WATTSUSED,WATTSUSEDNIGHTTARIFF PUMP1,PUMP2,VALVE,EL_HEATER,POWERBYBATTERY,  */
    sprintf( data, "%2d, %6.3f,%6.3f,%6.3f,%6.3f,%6.3f, %2d,%2d,%4.1f,%d, %5.3f,%5.3f, WANTED:",\
    current_timer_hour, Tkotel, Tkolektor, TboilerLow, TboilerHigh, Tenv, cfg.wanted_T, cfg.abs_max, \
    furnace_water_target, cfg.night_boost, TotalPowerUsed, NightlyPowerUsed );
    if ((HM&1)==1) sprintf( data + strlen(data), " P1");
    if ((HM&2)==2) sprintf( data + strlen(data), " P2");
    if ((HM&4)==4) sprintf( data + strlen(data), " V");
    if ((HM&8)==8) sprintf( data + strlen(data), " H");
    if ((HM&16)==16) sprintf( data + strlen(data), " Hf");
    if ((HM&32)==32) sprintf( data + strlen(data), " C1");
    if ((HM&64)==64) sprintf( data + strlen(data), " C2");
    if ((HM&128)==128) sprintf( data + strlen(data), " C4");
    sprintf( data + strlen(data), "; GOT:");
    if (CPump1) sprintf( data + strlen(data), " P1");
    if (CPump2) sprintf( data + strlen(data), " P2");
    if (CValve) sprintf( data + strlen(data), " V");
    if (CHeater) sprintf( data + strlen(data), " H");
    if (CCommsPin1) sprintf( data + strlen(data), " C1");
    if (CCommsPin2) sprintf( data + strlen(data), " C2");
    if (CCommsPin3) sprintf( data + strlen(data), " C3");
    if (CCommsPin4) sprintf( data + strlen(data), " C4");
    if (CPowerByBattery) sprintf( data + strlen(data), " UPS");
    if ((HM&256)==256) sprintf( data + strlen(data), " idle");
    log_message(DATA_FILE, data);

    sprintf( data, ",Temp1,%5.3f\n_,Temp2,%5.3f\n_,Temp3,%5.3f\n_,Temp4,%5.3f\n_,Temp5,%5.3f\n"\
    "_,Pump1,%d\n_,Pump2,%d\n_,Valve,%d\n_,Heater,%d\n_,PoweredByBattery,%d\n"\
    "_,TempWanted,%d\n_,BoilerTabsMax,%d\n_,ElectricityUsed,%5.3f\n_,ElectricityUsedNT,%5.3f",\
    Tkotel, Tkolektor, TboilerHigh, TboilerLow, Tenv, CPump1, CPump2,\
    CValve, CHeater, CPowerByBattery, cfg.wanted_T, cfg.abs_max,\
    TotalPowerUsed, NightlyPowerUsed );
    log_msg_ovr(TABLE_FILE, data);

    sprintf( data, "{Tkotel:%5.3f,Tkolektor:%5.3f,TboilerH:%5.3f,TboilerL:%5.3f,Tenv:%5.3f,"\
    "PumpFurnace:%d,PumpSolar:%d,Valve:%d,Heater:%d,PoweredByBattery:%d,"\
    "TempWanted:%d,BoilerTabsMax:%d,ElectricityUsed:%5.3f,ElectricityUsedNT:%5.3f}",\
    Tkotel, Tkolektor, TboilerHigh, TboilerLow, Tenv, CPump1, CPump2,\
    CValve, CHeater, CPowerByBattery, cfg.wanted_T, cfg.abs_max,\
    TotalPowerUsed, NightlyPowerUsed );
    log_msg_cln(JSON_FILE, data);
}

unsigned short ValveIsFullyOpen() {
    if (CValve && (SCValve > 13)) return 1;
    else return 0;
}

unsigned short ValveIsFullyClosed() {
    if (!CValve && (SCValve > 15)) return 1;
    else return 0;
}

unsigned short CanTurnPump1On() {
    if (cfg.use_pump1 && (!CPump1) && (SCPump1 > 2)) return 1;
    else return 0;
}

unsigned short CanTurnPump1Off() {
    if (CPump1 && !CValve && !CCommsPin1 && (SCPump1 > 5) && (SCValve > 5)) return 1;
    else return 0;
}

unsigned short CanTurnPump2On() {
    if (cfg.use_pump2 && (!CPump2) && (SCPump2 > 2)) return 1;
    else return 0;
}

unsigned short CanTurnPump2Off() {
    if (CPump2 && (SCPump2 > 5)) return 1;
    else return 0;
}

unsigned short CanTurnValveOn() {
    if (!CValve && (SCValve > 5)) return 1;
    else return 0;
}

unsigned short CanTurnValveOff() {
    if (CValve && (SCValve > 17)) return 1;
    else return 0;
}

unsigned short CanTurnHeaterOn() {
    unsigned short i = 0;
    if (!CHeater && (SCHeater > 29)) { i=1; } else return 0;
    /* Do the check with config to see if its OK to use electric heater,
    for example: if its on "night tariff" - switch it on */
    /* Determine current time: */
    if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) {
            /* NIGHT TARIFF TIME */
            /* If heater use is allowed by config - turn it on */
            if (cfg.use_electric_heater_night) return i;
    }
    else {
            /* DAY TIME */
            /* If heater use is allowed by config - turn it on */
            if (cfg.use_electric_heater_day) return i;
    }
    return 0;
}

unsigned short CanTurnHeaterOff() {
    if ((CHeater) && (SCHeater > 59)) return 1;
    else return 0;
}

unsigned short CanTurnHeatPumpLowOn() {
    if (!CCommsPin1 && (SCCommsPin1 > 35) && CPump1 && (SCHeater > 2)) return 1;
    else return 0;
}

unsigned short CanTurnHeatPumpLowOff() {
    if (CCommsPin1 && (SCCommsPin1 > 59))  return 1;
    else return 0;
}

unsigned short CanTurnHeatPumpHighOn() {
    if (!CCommsPin2 && (SCCommsPin2 > 15) && CPump1 && (SCHeater > 2)) return 1;
    else return 0;
}

unsigned short CanTurnHeatPumpHighOff() {
    if (CCommsPin2 && (SCCommsPin2 > 59))  return 1;
    else return 0;
}

unsigned short CanTurnCommsPin4On() {
    return 1;
}

unsigned short CanTurnCommsPin4Off() {
    if (CCommsPin4 && (SCCommsPin4 > 17)) return 1;
    else return 0;
}

void TurnPump1Off()  { CPump1 = 0; SCPump1 = 0; }
void TurnPump1On()  { CPump1 = 1; SCPump1 = 0; }
void TurnPump2Off()  { CPump2  = 0; SCPump2 = 0; }
void TurnPump2On()  { CPump2  = 1; SCPump2 = 0; }
void TurnValveOff()    { CValve  = 0; SCValve = 0; }
void TurnValveOn()    { CValve  = 1; SCValve = 0; }
void TurnHeaterOff()  { CHeater = 0; SCHeater = 0; }
void TurnHeaterOn()  { CHeater = 1; SCHeater = 0; }
void TurnHeatPumpLowOff()  { CCommsPin1 = 0; SCCommsPin1 = 0; }
void TurnHeatPumpLowOn()  { CCommsPin1 = 1; SCCommsPin1 = 0; }
void TurnHeatPumpHighOff() { CCommsPin2 = 0; SCCommsPin2 = 0; }
void TurnHeatPumpHighOn() { CCommsPin2 = 1; SCCommsPin2 = 0; }
void TurnCommsPin4Off()      { CCommsPin4 = 0; SCCommsPin4 = 0; } 
void TurnCommsPin4On()      { CCommsPin4 = 1; SCCommsPin4 = 0; }

/* Return non-zero value on critical condition found based on current data in sensors[] */
short
CriticalTempsFound() {
    if (Tkotel > 68) return 1;
    if (TboilerHigh > 71) return 2;
    return 0;
}

short
BoilerNeedsHeat() {
    short ret = 0;
    if ( (TboilerHigh < TboilerHighPrev) && (TboilerHighPrev < (float)cfg.wanted_T) ) ret=1;
    if ( TboilerHigh < ((float)cfg.wanted_T - 1) ) ret=2;
    if ( TboilerLow < ((float)cfg.wanted_T - (now_is_winter==1 ? 5:12)) ) ret=3;
    return ret;
}

short
ComputeWantedState() {
    short StateDesired = 0;
    short wantP1on = 0;
    short wantP2on = 0;
    short wantVon = 0;
    short wantHon = 0;
    short mid_buf = 0;
    static char data[280];

    /* EVACUATED TUBES COLLECTOR: EXTREMES PROTECTIONS */
    /* If collector is below 7 C and its getting cold -	turn pump on to prevent freezing */
	if ((Tkolektor < 7)&&(Tenv < 3)) wantP2on = 1;
    /* Prevent ETC from boiling its work fluid away in case all heat targets have been reached
        and yet there is no use because for example the users are away on vacation */
    if (Tkolektor > 65) {
        wantVon = 1;
        /* And if valve has been open for ~1.5 minutes - turn furnace pump on */
        if (CValve && (SCValve > 8)) wantP1on = 1;
        /* And if valve has been open for 2 minutes - turn solar pump on */
        if (CValve && (SCValve > 11)) wantP2on = 1;
    }

    /* FURNACE PUMP OPERATION */
	/* Furnace is above 38 C - at these temps always run the pump */
	if (Tkotel > 38) { wantP1on = 1; }
	else {
		/* below 38 C - check if it is cold to see if we need to run furnace pump:
            if so - run furnace pump at least once every 10 minutes
            we check if it is cold by looking at solar pump idle state - in the cold (-10C)
            it runs at least once per 2 hours; so double that ;) */
		if ((Tenv < 8)&&(SCPump2 < (6*60*4))&&(!CPump1)&&(SCPump1 > (6*10))) wantP1on = 1;
	}
    /* Furnace is above 20 C and rising slowly - turn pump on */
    if ((Tkotel > 20)&&(Tkotel > (TkotelPrev+0.12))) wantP1on = 1;
    /* Furnace temp is rising QUICKLY - turn pump on to limit furnace thermal shock */
    if (Tkotel > (TkotelPrev+0.18)) wantP1on = 1;

    /* BOILER HEATING: ALTERNATIVE SOURCES */
    /* Do the next checks for boiler heating if boiler is allowed to take heat in */
    if ( (TboilerHigh < (float)cfg.abs_max) || (TboilerLow < (float)(cfg.abs_max - 2)) ) {
        /* ETCs have heat in excess - build up boiler temp so expensive sources stay idle */
        /* Require selected heat source to be near boiler hot end to avoid loosing heat
        to the enviroment because of the system working */
        if ((Tkolektor > (TboilerLow+12))&&(Tkolektor > (TboilerHigh-2))) wantP2on = 1;
        /* Keep solar pump on while solar fluid is more than 5 C hotter than boiler lower end */
        if ((CPump2) && (Tkolektor > (TboilerLow+4))) wantP2on = 1;
        /* Furnace has heat in excess - open the valve so boiler can build up heat while it can */
        if ((Tkotel > (TboilerHigh+3)) || (Tkotel > (TboilerLow+9)))  {
            wantVon = 1;
            /* And if valve has been open for 90 seconds - turn furnace pump on */
            if (CValve && (SCValve > 8)) wantP1on = 1;
        }
        /* Keep valve open while there is still heat to exploit */
        if ((CValve) && (Tkotel > (TboilerLow+4))) wantVon = 1;
    }

    /* EVACUATED TUBES COLLECTOR: HOUSE KEEPING */
    /* Run solar pump once every day at the predefined hour for current month (see array definition)
    if it stayed off the past 4 hours*/
    if ( (current_timer_hour == pump_start_hour_for[current_month]) && (!CPump2) && (SCPump2 > (6*60*4)) )  {
        wantP2on = 1;
    }
        
    /* FURNACE PUMP: HOUSE KEEPING */
    if (cfg.pump1_always_on) {
        wantP1on = 1;
    }  else {
        /* Turn furnace pump on every 4 days */
        if ( (!CPump1) && (SCPump1 > (6*60*24*4)) ) wantP1on = 1;
    }

    /* ELECTRICAL HEATER: SMART FUNCTIONS */
    /* Two energy saving functions follow (if activated): */
    /* 1) During night tariff hours, try to keep boiler lower end near wanted temp */
    if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) {
        if ( (TboilerLow < ((float)cfg.wanted_T - 1.1)) && CanTurnHeaterOn() ) { wantHon = 1; }
    }
    /* 2) In the last 2 hours of night energy tariff heat up boiler until the lower sensor
    reads 12 C on top of desired temp, clamped at cfg.abs_max, so that less day energy gets used */
    if ( (cfg.night_boost) && (current_timer_hour >= (NEstop-1)) && (current_timer_hour <= NEstop) ) {
        if ((TboilerLow < nightEnergyTemp) && CanTurnHeaterOn()) { wantHon = 1; }
    }

    /* ELECTRICAL HEATER: BULK HEATING */
    if ( BoilerNeedsHeat() || wantHon ) {
        /* mark that we want the electrical heater on for deciding a bit later on */
        mid_buf = 1;
        wantHon = 0;
    }

    /* Decide wheter to request heet pump heat or not */
    if (Tkotel < furnace_water_target)  mid_buf += 2;
    sprintf( data, "compute: mb=%d", mid_buf );
    /* mid_buf holds our desired ON things - lets figure out what can be done in reality */
    if (mid_buf) {
        switch (cfg.max_big_consumers) {
        default:
        case 1: /* if only 1 big consumer allowed - check if boiler heater is needed */
            if (mid_buf == 1) { /* only boiler needs electrical heater */
                sprintf( data + strlen(data), " 1-1");
                /* check if other big consumers have been off at least 1 cycle */
                if ((!CCommsPin1 && (SCCommsPin1)) && (!CCommsPin2 && (SCCommsPin2))) {
                    /* and if we can turn heater ON */
                    sprintf( data + strlen(data), " check");
                    if (CanTurnHeaterOn()) wantHon = 1;
                    if (wantHon) sprintf( data + strlen(data), " OK!");
                }
            }
            if (mid_buf == 2) { /* we would like to use heat pump services */
                sprintf( data + strlen(data), " 1-2");
                /* check if other big consumers have been off at least 1 cycle */
                if ((!CHeater && (SCHeater)) && (!CCommsPin2 && (SCCommsPin2))) {
                    /* and if we can turn heater ON */
                    sprintf( data + strlen(data), " check");
                    if (CanTurnHeatPumpLowOn()) { StateDesired |= 32; }
                    if (StateDesired & 32) sprintf( data + strlen(data), " OK!");
                }
            }
            /* mid_buff == 3   => does not work - not enough power budget left */
        break;
        case 2: /* 2 BIG CONSUMERS*/
            if (mid_buf == 1) { /* only boiler needs electrical heater */
                sprintf( data + strlen(data), " 2-1");
                /* check if HP HIGH has been off and HP LOW - settled */
                if ( (SCCommsPin1) && (!CCommsPin2 && (SCCommsPin2))) {
                    /* and if we can turn heater ON */
                    sprintf( data + strlen(data), " check");
                    if (CanTurnHeaterOn()) wantHon = 1;
                    if (wantHon) sprintf( data + strlen(data), " OK!");
                }
            }
            if (mid_buf == 2) { /* we would like to use heat pump services */
                sprintf( data + strlen(data), " 2-2");
                /* check if HP HIGH has been OFF and heater - settled */
                if ((SCHeater) && (!CCommsPin2 && (SCCommsPin2))) {
                     /* verify rules following */
                    sprintf( data + strlen(data), " check");
                     if (CanTurnHeatPumpLowOn()) { StateDesired |= 32; }
                     if (CanTurnHeatPumpHighOn()) { StateDesired |= 64; }
                    if (StateDesired & 32) sprintf( data + strlen(data), " OK1!");
                    if (StateDesired & 64) sprintf( data + strlen(data), " OK2!");
                 }
            }
            if (mid_buf == 3) { /* we would like to use BOTH heat pump and heater */
                sprintf( data + strlen(data), " 2-3");
                /* give boiler priority if possible - HP LOW HP HIGH needs to be off settled and HP HIGH - off */
                if ((SCCommsPin1>2) && (!CCommsPin2 && (SCCommsPin2))) {
                    /* verify rules following */
                    sprintf( data + strlen(data), " check1");
                    if (CanTurnHeaterOn()) wantHon = 1;
                    if (wantHon) sprintf( data + strlen(data), " OK!");
                }
                /* for HP LOW - we can keep it on as long as it follows its rules */
                if (CanTurnHeatPumpLowOn()) { StateDesired |= 32; }
                if (StateDesired & 32) sprintf( data + strlen(data), " OK1!");
                /* for HP HIGH - boiler should not be needed and others should have settled */
                if (!wantHon && (SCCommsPin1>2) && (!CCommsPin2 && (SCCommsPin2))) {
                    /* check if turning HP high follows its rules */
                sprintf( data + strlen(data), " check3");
                    if (CanTurnHeatPumpHighOn()) { StateDesired |= 64; }
                    if (StateDesired & 64) sprintf( data + strlen(data), " OK2!");
                }
            }
        break;
        case 3: /* 3 big consumers allowed - you got thick cables, so we do not care what we turn on */
            if (mid_buf == 1) { /* only boiler needs electrical heater */
                sprintf( data + strlen(data), " 3-1");
                /* avoid simultaneous switching */
                if ((SCCommsPin1) && (SCCommsPin2>2)) {
                    /* verify rules following */
                    if (CanTurnHeaterOn()) wantHon = 1;
                }
            }
            if (mid_buf == 2) { /* we would like to use heat pump services */
                sprintf( data + strlen(data), " 3-2");
                /* avoid simultaneous switching */
                if ( SCHeater && SCCommsPin1 && (SCCommsPin2>3)) {
                    /* verify rules following */
                     if (CanTurnHeatPumpLowOn()) { StateDesired |= 32; }
                     if (CanTurnHeatPumpHighOn()) { StateDesired |= 64; }
                 }
            }
            if (mid_buf == 3) { /* we would like to use BOTH heat pump and heater */
                sprintf( data + strlen(data), " 3-3");
                /* avoid simultaneous switching */
                if ((SCCommsPin1) && (SCCommsPin2>2)) {
                    /* verify rules following */
                    if (CanTurnHeaterOn()) wantHon = 1;
                }
                /* avoid simultaneous switching */
                if ( SCHeater && SCCommsPin1 && (SCCommsPin2>3)) {
                    /* verify rules following */
                     if (CanTurnHeatPumpLowOn()) { StateDesired |= 32; }
                     if (CanTurnHeatPumpHighOn()) { StateDesired |= 64; }
                 }
            }
        break;
        }
        /* after the swtich above - request pump 1 only if needed */
        if ( StateDesired & 32 ) wantP1on = 1;
    }
    
    log_message(DATA_FILE, data);
    
    if ( wantP1on ) StateDesired |= 1;
    if ( wantP2on ) StateDesired |= 2;
    if ( wantVon )  StateDesired |= 4;
    if ( wantHon )  StateDesired |= 8;
    if ( CPowerByBattery )  StateDesired |= 128;

    return StateDesired;
}

void
ActivateDevicesState(const short HeatMode) {
    char current_state = 0;
    char new_state = 0;

    /* calculate current state */
    if ( CPump1 ) current_state |= 1;
    if ( CPump2 ) current_state |= 2;
    if ( CValve ) current_state |= 4;
    if ( CHeater ) current_state |= 8;
    if ( CCommsPin1 ) current_state |= 32;
    if ( CCommsPin2 ) current_state |= 64;
    if ( CCommsPin4 ) current_state |= 128;
    /* make changes as needed */
    /* HeatMode's bits describe the peripherals desired state:
        bit 1  (1) - pump 1
        bit 2  (2) - pump 2
        bit 3  (4) - valve
        bit 4  (8) - heater wanted
        bit 5 (16) - heater forced
        bit 6 (32) - want heat pump LOW on
        bit 7 (64) - want heat pump HIGH on */
    if (HeatMode &   1)  { if (CanTurnPump1On()) TurnPump1On(); } else { if (CanTurnPump1Off()) TurnPump1Off(); }
    if (HeatMode &   2)  { if (CanTurnPump2On()) TurnPump2On(); } else { if (CanTurnPump2Off()) TurnPump2Off(); }
    if (HeatMode &   4)  {  if (CanTurnValveOn()) TurnValveOn(); } else {  if (CanTurnValveOff()) TurnValveOff(); }
    if (HeatMode &   8)  {  if (CanTurnHeaterOn()) TurnHeaterOn(); }
    if (HeatMode &  16) { TurnHeaterOn(); }
    if ( !(HeatMode & 24) ) { if (CanTurnHeaterOff()) TurnHeaterOff(); }
    if (HeatMode &  32)  { if (CanTurnHeatPumpLowOn()) TurnHeatPumpLowOn(); } else { if (CanTurnHeatPumpLowOff()) TurnHeatPumpLowOff(); }
    if (HeatMode &  64)  { if (CanTurnHeatPumpHighOn()) TurnHeatPumpHighOn(); } else { if (CanTurnHeatPumpHighOff()) TurnHeatPumpHighOff(); }
    if (HeatMode & 128) { if (CanTurnCommsPin4On()) TurnCommsPin4On(); } else { if (CanTurnCommsPin4Off()) TurnCommsPin4Off(); }
    
    SCPump1++;
    SCPump2++;
    SCValve++;
    SCHeater++;
    SCCommsPin1++;
    SCCommsPin2++;
    SCCommsPin3++;
    SCCommsPin4++;

    /* Calculate total and night tariff electrical power used here: */
    if ( CHeater ) {
        TotalPowerUsed += HEATERPPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += HEATERPPC; }
    }
    if ( CPump1 ) {
        TotalPowerUsed += PUMP1PPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += PUMP1PPC; }
    }
    if ( CPump2 ) {
        TotalPowerUsed += PUMP2PPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += PUMP2PPC; }
    }
    if ( CValve ) {
        TotalPowerUsed += VALVEPPC;
        if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += VALVEPPC; }
    }
    TotalPowerUsed += SELFPPC;
    if ( (current_timer_hour <= NEstop) || (current_timer_hour >= NEstart) ) { NightlyPowerUsed += SELFPPC; }

    /* calculate desired new state */
    if ( CPump1 ) new_state |= 1;
    if ( CPump2 ) new_state |= 2;
    if ( CValve ) new_state |= 4;
    if ( CHeater ) new_state |= 8;
    if ( CCommsPin1 ) new_state |= 32;
    if ( CCommsPin2 ) new_state |= 64;
    if ( CCommsPin4 ) new_state |= 128;
    /* if current state and new state are different... */
    if ( current_state != new_state ) {
        /* then put state on GPIO pins - this prevents lots of toggling at every 10s decision */
        ControlStateToGPIO();
    }
}

void
AdjustWantedStateForBatteryPower(unsigned short WS) {
    /* Check for power source switch */
    if ( CPowerByBattery != CPowerByBatteryPrev ) {
        /* If we just switched to battery.. */
        if ( CPowerByBattery ) {
            log_message(LOG_FILE,"WARNING: Switch to BATTERY POWER detected.");
        }
        else {
            log_message(LOG_FILE,"INFO: Powered by GRID now.");
        }
    }
    /* in the first 10 minutes of battery power there is a high chance that power will be back;
    prepare for this by keeping the boiler electrical heater ON and ready to switch it off */
    if ( CPowerByBattery ) {
        /* When battery powered - force electrical heater ON */
        WS |= 16;
        /* enable quick heater turn off */
        SCHeater = 30;
    }
}

int
main(int argc, char *argv[])
{
    /* set iter to its max value - makes sure we get a clock reading upon start */
    unsigned short iter = 30;
    unsigned short iter_P = 0;
    unsigned short AlarmRaised = 0;
    unsigned short DevicesWantedState = 0;
    struct timeval tvalBefore, tvalAfter;

    SetDefaultCfg();

    /* before main work starts - try to open the log files to write a new line
    ...and SCREAM if there is trouble! */
    if (log_message(LOG_FILE,"***")) {
        printf("Cannot open the mandatory "LOG_FILE" file needed for operation!\n");
        exit(3);
    }
    if (log_message(DATA_FILE,"***")) {
        printf("Cannot open the mandatory "DATA_FILE" file needed for operation!\n");
        exit(4);
    }
    if (log_message(TABLE_FILE,"***")) {
        printf("Cannot open the mandatory "TABLE_FILE" file needed for operation!\n");
        exit(5);
    }
    if (log_message(JSON_FILE,"***")) {
        printf("Cannot open the mandatory "JSON_FILE" file needed for operation!\n");
        exit(6);
    }
    if (log_message(CFG_TABLE_FILE,"***")) {
        printf("Cannot open the mandatory "CFG_TABLE_FILE" file needed for operation!\n");
        exit(7);
    }

    daemonize();

    write_log_start();

    just_started = 3;
    TotalPowerUsed = 0;
    NightlyPowerUsed = 0;

    parse_config();

    ReadPersistentPower();

    /* Enable GPIO pins */
    if ( ! EnableGPIOpins() ) {
        log_message(LOG_FILE,"ALARM: Cannot enable GPIO! Aborting run.");
        exit(11);
    }

    /* Set GPIO directions */
    if ( ! SetGPIODirection() ) {
        log_message(LOG_FILE,"ALARM: Cannot set GPIO direction! Aborting run.");
        exit(12);
    }

    do {
        /* Do all the important stuff... */
        if ( gettimeofday( &tvalBefore, NULL ) ) {
            log_message(LOG_FILE,"WARNING: error getting tvalBefore...");
        }
        /* get the current hour every 5 minutes for electric heater schedule */
        if ( iter == 30 ) {
            iter = 0;
            GetCurrentTime();
            /* and increase counter controlling writing out persistent power use data */
            iter_P++;
            if ( iter_P == 2) {
                iter_P = 0;
                WritePersistentPower();
            }
        }
        iter++;
        ReadSensors();
        ReadExternalPower();
        /* do what "mode" from CFG files says - watch the LOG file to see used values */
        switch (cfg.mode) {
            default:
            case 0: /* 0=ALL OFF */
            DevicesWantedState = 0;
            break;
            case 1: /* 1=AUTO - tries to reach desired water temp efficiently */
            if ( CriticalTempsFound() ) {
                /* ActivateEmergencyHeatTransfer(); */
                /* Set DevicesWantedState bits for both pumps and valve */
                DevicesWantedState = 1 + 2 + 4;
                if ( !AlarmRaised ) {
                    log_message(LOG_FILE,"ALARM: Activating emergency cooling!");
                    AlarmRaised = 1;
                }
            }
            else {
                if ( AlarmRaised ) {
                    log_message(LOG_FILE,"INFO: Critical condition resolved. Running normally.");
                    AlarmRaised = 0;
                }
                DevicesWantedState = ComputeWantedState();
            }
            break;
        }
        AdjustWantedStateForBatteryPower(DevicesWantedState);
        ActivateDevicesState(DevicesWantedState);
        /* for the first 3 cycles  = 30 seconds - do not log anything */
        if ( ProgramRunCycles > 2) { LogData(DevicesWantedState); }
        ProgramRunCycles++;
        if ( just_started ) { just_started--; }
        if ( need_to_read_cfg ) {
            need_to_read_cfg = 0;
            just_started = 1;
            parse_config();
        }
        if ( gettimeofday( &tvalAfter, NULL ) ) {
            log_message(LOG_FILE,"WARNING: error getting tvalAfter...");
            sleep( 7 );
        }
        else {
            /* use hardcoded sleep() if time is skewed (for eg. daylight saving, ntp adjustments, etc.) */
            if ((tvalAfter.tv_sec - tvalBefore.tv_sec) > 12) {
                sleep( 7 );
            }
            else {
                /* otherwise we have valid time data - so calculate exact sleep time
                so period between active operations is bang on 10 seconds */
                usleep(10000000 - (((tvalAfter.tv_sec - tvalBefore.tv_sec)*1000000L \
                + tvalAfter.tv_usec) - tvalBefore.tv_usec));
            }
        }
    } while (1);

    /* Disable GPIO pins */
    if ( ! DisableGPIOpins() ) {
        log_message(LOG_FILE,"ALARM: Cannot disable GPIO on UNREACHABLE exit!");
        return(222);
    }

    log_message(LOG_FILE,"ALARM: Something wierd happened. Reached an UNREACHABLE part of the program!");

    return(225);
}

/* EOF */
