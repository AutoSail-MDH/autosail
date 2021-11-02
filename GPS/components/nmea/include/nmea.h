#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <regex.h>

#define NMEA_MESSAGES "RMC|GLL|GGA"
#define LONG_LAT "[0-9]{4}\\.[0-9]{4}"

#ifndef HEADER_NMEA
#define HEADER_NMEA

int match(char * buf, char * pattern, regmatch_t * pmatch);
int getPos(char * buf, float * lon, float * lat);
int parse(char * buf, char * pattern, float * lon, float * lat);

#endif