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
    char tmp_time[10];
    char tmp_lat[11];
    char tmp_long[12];
    char tmp_fix[2];
    if ((strncmp(buf, "$GP", 3) == 0) |
        (strncmp(buf, "$GN", 3) == 0)) {
        if (strncmp(&buf[3], "GGA", 3) == 0) {
            /*
            i = parse_comma_delimited_str(buf, field, 20);
            *timestamp = strtof(field[1], NULL);
            *lat = strtof(field[2], NULL);
            *lon = strtof(field[4], NULL);
            *gps_fix = atoi(field[6]);
            */
           
            strncpy(tmp_time, &buf[7], 9);
            strncpy(tmp_lat, &buf[17], 10);
            strncpy(tmp_long, &buf[30], 11);
            strncpy(tmp_fix, &buf[44], 1);

            *timestamp = roundf(10 * strtof(tmp_time, NULL)) / 10;
            *lat = strtof(tmp_lat, NULL);
            *lon = strtof(tmp_long, NULL);
            *gps_fix = 1;
            //*gps_fix = atoi(tmp_fix);
            
            return 1;
        }
        else{
            *gps_fix = 0;
            return -1;
        }
    }

    return 0;
}
