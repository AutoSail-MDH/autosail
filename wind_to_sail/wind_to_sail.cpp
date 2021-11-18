#include <iostream>
using namespace std;

int main() {
    int wind_direction = 0;
    float wind_angle_rad = -3.1415926536;
    float wind_angle = 0;
    float sail_angle = 0;
    float pi = 3.141592653589793238463;  // 3.14159;  // 22 / 7;

    // convert radians to degrees
    wind_angle = wind_angle_rad * (180 / pi);

    // determine which wind direction the wind angle is
    if (wind_angle > -45 && wind_angle < 45) {
        wind_direction = 1;
    } else if ((wind_angle >= 45 && wind_angle < 55) || (wind_angle > -55 && wind_angle <= -45)) {
        wind_direction = 2;
    } else if ((wind_angle >= 55 && wind_angle < 80) || (wind_angle > -80 && wind_angle <= -55)) {
        wind_direction = 3;
    } else if ((wind_angle >= 80 && wind_angle < 105) || (wind_angle > -105 && wind_angle <= -80)) {
        wind_direction = 4;
    } else if ((wind_angle >= 105 && wind_angle < 170) || (wind_angle > -170 && wind_angle <= -105)) {
        wind_direction = 5;
    } else if ((wind_angle >= 170 && wind_angle <= 180) || (wind_angle >= -180 && wind_angle <= -170)) {
        wind_direction = 6;
    }

    // set angle according to wind direction
    switch (wind_direction) {
        case 1:  // no go
            sail_angle = 0;
            break;
        case 2:  // close hauled
            sail_angle = 0;
            break;
        case 3:  // close reach
            if (wind_angle > 0) {
                sail_angle = 30;
            } else {
                sail_angle = -30;
            }
            break;
        case 4:  // beam reach
            if (wind_angle > 0) {
                sail_angle = 45;
            } else {
                sail_angle = -45;
            }
            break;
        case 5:  // broad reach
            if (wind_angle > 0) {
                sail_angle = 60;
            } else {
                sail_angle = -60;
            }
            break;
        case 6:  // running
            if (wind_angle > 0) {
                sail_angle = 90;
            } else {
                sail_angle = -90;
            }
            break;
        default:
            break;
    }
    cout << "Wind direction: " << wind_angle << " Sail angle: " << sail_angle << endl;

    return 0;
}