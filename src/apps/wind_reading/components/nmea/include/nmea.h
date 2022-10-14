#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <regex.h>

//This pattern matches any string that contains either RMC, GLL or GGA anywhere in the string
#define NMEA_MESSAGES "MWV"
//This pattern matches any string that has the format dddd where d is an integer anywhere in the string
#define SPEED "[0-9]{3}\\.[0-9]{1}"
#define ANGLE "[0-9]{3}"

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
 * @param lat A pointer to the latitude value, as a float. Has the value 0 on failure
 * @param lon A pointer to the longitude value, as a float. Has the value 0 on failure
 * @return 
 *      - 1 on SUCCESS
 *      - 0 on FAILURE
 */

int getWind(char * buf, float * angle, float * speed);

/**
 * @brief Finds the long/lat values in an NMEA message
 * 
 * @param buf A char pointer to a string containing the NMEA message
 * @param pattern The regex pattern
 * @param lat A pointer to the latitude value, as a float. Has the value 0 on failure
 * @param lon A pointer to the longitude value, as a float. Has the value 0 on failure
 * @return
 *      - 1 on SUCCESS
 *      - 0 on FAILURE
 * 
 */
int parse(char * buf, char * pattern_angle, char * pattern_speed, float * angle, float * speed);

#endif