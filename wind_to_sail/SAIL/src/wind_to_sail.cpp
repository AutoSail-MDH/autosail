#include <iostream>

#include "sail.cpp"

using namespace std;

int main() {
    int wind_direction = 0;
    float wind_angle_rad = -3.1415926536;
    float wind_angle = 0;
    float sail_angle = 0;
    float pi = 3.14159;  // 3.141592653589793238463;  // 3.14159;  // 22 / 7;

    // convert radians to degrees
    wind_angle = wind_angle_rad * (180 / pi);

    // determine which wind direction the wind angle is
    wind_direction = get_direction(wind_angle);

    // set angle according to wind direction
    sail_angle = set_angle(wind_direction, wind_angle);

    cout << "Wind direction: " << wind_angle << " Sail angle: " << sail_angle << endl;

    return 0;
}