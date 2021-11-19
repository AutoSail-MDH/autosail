#include "sail.cpp"

int main() {
    int wind_direction = 0;
    float wind_angle = 60;
    float sail_angle = 0;

    // determine which wind direction the wind angle is
    wind_direction = get_direction(wind_angle);

    // set angle according to wind direction
    sail_angle = set_angle(wind_direction, wind_angle);

    cout << "Wind direction: " << wind_angle << " Sail angle: " << sail_angle << endl;

    return 0;
}