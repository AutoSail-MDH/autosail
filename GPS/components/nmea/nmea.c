#include "nmea.h"

int match(char * buf, char * pattern, regmatch_t * pmatch){
    regex_t    preg;
    int        rc;
    size_t     nmatch = 2;

    if (0 != (rc = regcomp(&preg, pattern, REG_EXTENDED))) {
        //printf("regcomp() failed, returning nonzero (%d)\n", rc);
        exit(EXIT_FAILURE);
    }

    if (0 != (rc = regexec(&preg, buf, nmatch, pmatch, 0))) {
        //printf("Failed to match '%s' with '%s',returning %d.",
        //        buf, pattern, rc);
        regfree(&preg);
        return 0;
    }

    regfree(&preg);
    return 1;
}

int parse(char * buf, char * pattern, float * lon, float * lat){
    regmatch_t pmatch[2];

    if(!match(buf, pattern, pmatch)){
        return 0;
    }
    
    *lon = strtof(&buf[pmatch[0].rm_so], NULL)/100;

    int start = pmatch[0].rm_eo + 1;

    if(!match(&buf[start], pattern, pmatch)){
        return 0;
    }
    *lat = strtof(&buf[start + pmatch[0].rm_so], NULL)/100;

    return 1;
}

int getPos(char * buf, float * lon, float * lat){
    regmatch_t tmp[1];

    if(!match(buf, NMEA_MESSAGES, tmp)){
        return 0;
    }
    if(!parse(buf, LONG_LAT, lon, lat)){
        return 0;
    }

    return 1;
}
