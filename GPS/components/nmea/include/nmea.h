#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <regex.h>

#define NMEA_MESSAGES "RMC|GLL|GGA"
#define LONG_LAT "[0-9]{4}\\.[0-9]{4}"

#ifndef HEADER_NMEA
#define HEADER_NMEA

/**
 * @brief Matches a string to a regex pattern
 * 
 * @param buf A char pointer to a string
 * @param pattern Regex pattern for matching
 * @param pmatch Struct that contains where the match was found
 * @return 
 *      - 1 on SUCCESS
 *      - 0 on FAILURE
 */

int match(char * buf, char * pattern, regmatch_t * pmatch);

/**
 * @brief Gets data from an NMEA message
 * 
 * @param buf A char pointer to a string containing the NMEA message
 * @param lon A pointer to the longitude value, as a float
 * @param lat A pointer to the latitude value, as a float
 * @return 
 *      - 1 on SUCCESS
 *      - 0 on FAILURE
 */

int getPos(char * buf, float * lon, float * lat);

/**
 * @brief Finds the long/lat values in an NMEA message
 * 
 * @param buf A char pointer to a string containing the NMEA message
 * @param pattern The regex pattern
 * @param lon A pointer to the longitude value, as a float
 * @param lat A pointer to the latitude value, as a float
 * @return
 *      - 1 on SUCCESS
 *      - 0 on FAILURE
 * 
 */
int parse(char * buf, char * pattern, float * lon, float * lat);

#endif