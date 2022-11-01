#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#ifndef HEADER_NMEA
#define HEADER_NMEA

int parse_comma_delimited_str(char * string, char ** fields, int max_fields);

int get_position(char * buf, float * timestamp, float * lat, float * lon, int * gps_fix);

#endif