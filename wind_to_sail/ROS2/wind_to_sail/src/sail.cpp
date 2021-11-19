#include "sail.hpp"

int get_direction(float wind_angle) {
    int wind_direction = 0;

    if (wind_angle < 45 && wind_angle > 315) {
        wind_direction = 1;
    } else if ((wind_angle >= 45 && wind_angle < 55) || (wind_angle > 305 && wind_angle <= 315)) {
        wind_direction = 2;
    } else if ((wind_angle >= 55 && wind_angle < 80) || (wind_angle > 280 && wind_angle <= 305)) {
        wind_direction = 3;
    } else if ((wind_angle >= 80 && wind_angle < 105) || (wind_angle > 255 && wind_angle <= 280)) {
        wind_direction = 4;
    } else if ((wind_angle >= 105 && wind_angle < 170) || (wind_angle > 190 && wind_angle <= 255)) {
        wind_direction = 5;
    } else if (wind_angle >= 170 && wind_angle <= 190) {
        wind_direction = 6;
    }
    return wind_direction;
}

// for [-180,180]
/*if (wind_angle > -45 && wind_angle < 45) {
    wind_direction = 1;
} else if ((wind_angle >= 45 && wind_angle < 55) || (wind_angle > -55 && wind_angle <= -45)) { 55-305 45-315
    wind_direction = 2;
} else if ((wind_angle >= 55 && wind_angle < 80) || (wind_angle > -80 && wind_angle <= -55)) {80-280
    wind_direction = 3;
} else if ((wind_angle >= 80 && wind_angle < 105) || (wind_angle > -105 && wind_angle <= -80)) {105-255
    wind_direction = 4;
} else if ((wind_angle >= 105 && wind_angle < 170) || (wind_angle > -170 && wind_angle <= -105)) {170-190
    wind_direction = 5;
} else if ((wind_angle >= 170 && wind_angle <= 180) || (wind_angle >= -180 && wind_angle <= -170)) {180-180
    wind_direction = 6;
}*/

float set_angle(int wind_direction, float wind_angle) {
    float sail_angle = 0;
    switch (wind_direction) {
        case 1:  // no go
            sail_angle = 0;
            break;
        case 2:  // close hauled
            sail_angle = 0;
            break;
        case 3:  // close reach
            if (wind_angle < 180) {
                sail_angle = 30;
            } else {
                sail_angle = -30;
            }
            break;
        case 4:  // beam reach
            if (wind_angle < 180) {
                sail_angle = 45;
            } else {
                sail_angle = -45;
            }
            break;
        case 5:  // broad reach
            if (wind_angle < 180) {
                sail_angle = 60;
            } else {
                sail_angle = -60;
            }
            break;
        case 6:  // running
            if (wind_angle < 180) {
                sail_angle = 90;
            } else {
                sail_angle = -90;
            }
            break;
        default:
            break;
    }
    return sail_angle;
}