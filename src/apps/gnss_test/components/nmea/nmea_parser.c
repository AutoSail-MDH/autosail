#include "include/nmea_parser.h"

int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
   int i = 0;
   fields[i++] = string;
   while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
      *string = '\0';
      fields[i++] = ++string;
   }
   return --i;
}


int get_position(char * buf, float * timestamp, float * lat, float * lon, int * gps_fix) 
{
    int i = 0;
    char *field[20];
    if ((strncmp(buf, "$GP", 3) == 0) |
        (strncmp(buf, "$GN", 3) == 0)) {
        if (strncmp(&buf[3], "GGA", 3) == 0) {
            i = parse_comma_delimited_str(buf, field, 20);
            *timestamp = strtof(field[1], NULL);
            *lat = strtof(field[2], NULL);
            *lon = strtof(field[4], NULL);
            *gps_fix = atoi(field[6]);
            return 1;
        }
    }

    return 0;
}
