#include "include/nmea.h"

int match(char* buf, char* pattern, regmatch_t* pmatch) {
    regex_t preg;
    int rc;
    size_t nmatch = 2;

    // Creates a regex variable that is configured for the pattern specified.
    if (0 != (rc = regcomp(&preg, pattern, REG_EXTENDED))) {
        return 0;
    }
    // Applies the pattern to a string as specified by buf. pmatch returns information about where the match happened
    if (0 != (rc = regexec(&preg, buf, nmatch, pmatch, 0))) {
        regfree(&preg);
        return 0;
    }

    regfree(&preg);
    return 1;
}

int parse(char* buf, char* pattern_1, char* pattern_2, float* angle, float* speed) {
    regmatch_t pmatch[2];

    // Tries to match according to a pattern
    if (!match(buf, pattern_1, pmatch)) {
        return 0;
    }
    // Converts the char value to a float
    *angle = strtof(&buf[pmatch[0].rm_so], NULL);

    int start = pmatch[0].rm_eo + 1;

    // Tries to match again on the new string, which is everything in the string that is after the earlier match
    if (!match(&buf[start], pattern_2, pmatch)) {
        return 0;
    }
    // Converts the char value to a float
    *speed = strtof(&buf[start + pmatch[0].rm_so], NULL);

    return 1;
}

int getWind(char* buf, float* angle, float* speed) {
    regmatch_t pmatch[2];

    // Tries to find out if the message is either GGA, GLL or RMC, which measn they contain positions
    if (!match(buf, NMEA_MESSAGES, pmatch)) {
        return 0;
    }
    if (pmatch[0].rm_so < 10) {
        // Tries to find out where the long/lat values are and populate he lon/lat
        if (!parse(buf, ANGLE, SPEED, angle, speed)) {
            return 0;
        }
        return 1;
    }

    return 0;
}
