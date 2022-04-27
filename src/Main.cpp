
#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <loguru/loguru.hpp>
#include <math.h>

using namespace std;

#define EARTH_RADIUS 6371

uint32_t blc_test[8]={12.0333,3.55456,10200,0,0,0,0,45.3};//{lat/lon/alt/relative_alt/vx/vy/vz/hdg}
float command_motors[4];//{x,y,z,r}
float target_coordinates[4]={2034.32,6352.65,5,};//{x,y,z,r}


void convert_polar_to_cartesian(float lat, float lon, float alt, float& x,float& y, float& z);
void calculate_rotation(float hdg, float targeted_hdg, float& r);
void calculate_vector(float x_drone, float y_drone, float z_drone, float x_target, float y_target, float z_target, float& x, float& y, float& z);

int main(int argc, char* argv[])
{
    srand(time(0));
    loguru::init(argc, argv);
    LOG_F(INFO, "Start the test autopilot program");

    float x, y, z;
    convert_polar_to_cartesian(blc_test[0], blc_test[1], blc_test[2], x, y, z);
    calculate_vector(x, y, z, target_coordinates[0], target_coordinates[0], target_coordinates[0], command_motors[0], command_motors[1], command_motors[2]);
    calculate_rotation(blc_test[7],target_coordinates[4],command_motors[4]);


    LOG_F(INFO, "End of the program test autopilot");
    return EXIT_SUCCESS;
}

void convert_polar_to_cartesian(float lat, float lon, float alt, float& x,float& y, float& z)
{
    x = EARTH_RADIUS * cos(lat) * cos(lon);
    y = EARTH_RADIUS * cos(lat) * sin(lon);
    z = alt*1000;
}

void calculate_rotation(float hdg, float targeted_hdg, float& r)
{
    float theta = abs(hdg-targeted_hdg);

    if (theta<180)
    {
        r=-1;
    }else{
        r=1;
    }
    
}

void calculate_vector(float x_drone, float y_drone, float z_drone, float x_target, float y_target, float z_target, float& x, float& y, float& z)
{
    x= x_drone-x_target;
    y= y_drone-y_target;
    z= z_drone-z_target;
}

