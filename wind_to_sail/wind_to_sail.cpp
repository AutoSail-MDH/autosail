#include <iostream>
using namespace std;

int main() {
    int wind_direction = 0;
    float wind_angle = -45;

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
            cout << "1" << endl;
            break;
        case 2:  // close hauled
            cout << "2" << endl;
            break;
        case 3:  // close reach
            cout << "3" << endl;
            break;
        case 4:  // beam reach
            cout << "4" << endl;
            break;
        case 5:  // broad reach
            cout << "5" << endl;
            break;
        case 6:  // running
            cout << "6" << endl;
            break;
        default:
            cout << "0" << endl;
    }

    return 0;
}