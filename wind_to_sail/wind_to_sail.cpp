#include <iostream>
using namespace std;

int main() {
    int wind_direction = 0;
    float wind_angle = -45;
    float sail_angle = 0;

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

    switch (wind_direction) {
        case 1:  // no go
            cout << "no go" << endl;
            sail_angle = 0;
            break;
        case 2:  // close hauled
            cout << "close hauled" << endl;
            sail_angle = 0;
            break;
        case 3:  // close reach
            cout << "close reach" << endl;
            if (wind_angle < 0) {
                sail_angle = 30;
            } else {
                sail_angle = -30;
            }

            break;
        case 4:  // beam reach
            cout << "beam reach" << endl;
            if (wind_angle < 0) {
                sail_angle = 45;
            } else {
                sail_angle = -45;
            }

            break;
        case 5:  // broad reach
            cout << "broad reach" << endl;
            if (wind_angle < 0) {
                sail_angle = 60;
            } else {
                sail_angle = -60;
            }

            break;
        case 6:  // running
            cout << "running" << endl;
            sail_angle = 90;
            break;
        default:
            cout << "0" << endl;
    }
    cout << "Wind direction: " << wind_angle << " Sail angle: " << sail_angle << endl;

    return 0;
}