#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <loguru/loguru.hpp>
#include <math.h>

using namespace std;

#define EARTH_RADIUS 6371
#define MATH_PI 3.141592
#define RANGE 10000

struct Vector{
    float vx;
    float vy;
    float vz;
    float vr;
};

struct Informations{
    float ratiox = 0;
    float ratioy = 0;
    float ratioz = 0;
    float ratior = 0;
    bool inRange = false;
    bool isArrived = false;
    Vector vectors;
};

struct Coordinates{
    float latitude;
    float longitude;
    float altitude;
    float rotation;
};

float blc_test[8]={12.0333,3.55456,10200,0,0,0,0,45.3};//{lat/lon/alt/relative_alt/vx/vy/vz/hdg}
float command_motors[4];//{x,y,z,r}
float target_coordinates[4]={2034.32,6352.65,5,};//{x,y,z,r} mocked

void convert_polar_to_cartesian(float lat, float lon, float alt, float& x,float& y, float& z);
void calculate_rotation(float hdg, float targeted_hdg, float& r);
void calculate_vector(float x_drone, float y_drone, float z_drone, float x_target, float y_target, float z_target, float& x, float& y, float& z);
void convert_cartesian_to_polar(float x, float y, float z, float& lat, float& lon, float& alt);

void calculate_ratios(Coordinates droneCoordinates, Informations& infos);

int main(int argc, char* argv[])
{
    srand(time(0));
    struct Informations infos;
    struct Coordinates droneCoordinates;
    droneCoordinates.altitude = blc_test[2];
    droneCoordinates.longitude = blc_test[1];
    droneCoordinates.latitude = blc_test[0];
    droneCoordinates.rotation = blc_test[3];
    
    loguru::init(argc, argv);
    LOG_F(INFO, "Start the test autopilot program");
    calculate_ratios(droneCoordinates, infos);
    LOG_F(INFO, "Informations 5 (info struct): rx = %f | ry = %f | rz = %f | rr = %f | inRange = %d | isArrived = %d", infos.ratiox, infos.ratioy, infos.ratioz, infos.ratior, infos.inRange, infos.isArrived);
    LOG_F(INFO, "End of the program test autopilot");
    return EXIT_SUCCESS;
}

void calculate_ratios(Coordinates droneCoordinates, Informations& infos){

    /*
        Ce qui sera retourné
        infos : x, y, z, r (entre -1 et 1), inRange [disant si le drone est a portée] et isArrived [disant si le drone est bien arrivé]
    */

    // Coordonnées polaires du drone
    float latitude = droneCoordinates.latitude;
    float longitude = droneCoordinates.longitude;
    float altitude = droneCoordinates.altitude;
    float rotation = droneCoordinates.rotation;
    
    // Coordonnées cartésiennes du drone
    float px;
    float py;
    float pz;

    // Coordonnées polaires du point de destination
    float bLatitude = target_coordinates[0];
    float bLongitude = target_coordinates[1];
    float bAltitude = target_coordinates[2];
    float bRotation = target_coordinates[3];

    // Coordonées cartésiennes du point de destination
    float bx;
    float by;
    float bz;

    convert_polar_to_cartesian(latitude, longitude, altitude, px, py, pz);
    convert_polar_to_cartesian(bLatitude, bLongitude, bAltitude, bx, by, bz);
    
    LOG_F(INFO, "Informations 1 (Drone polar): lat: %f | long: %f | alt: %f", latitude, longitude, altitude);
    LOG_F(INFO, "Informations 2 (Drone cartesian): px: %f | py: %f | pz: %f", px, py, pz);
    LOG_F(INFO, "Informations 3 (But) : bx: %f | by: %f | bz: %f", bx, by, bz);

    bool isInRange = ((px > bx - RANGE && px < bx + RANGE) 
        && (py > by - RANGE && py < by + RANGE) 
            && (pz > bz - RANGE && pz < bz + RANGE));

    bool onPoint = (px == bx) && (py == by) && (pz == bz);

    // On effectue une comparaison afin de savoir dans quelle direction on doit se diriger en x, y et z
    if(isInRange){
        // On alerte qu'on est entré dans la sphère d'arrivée, les ratios sont définies par défaut à 0
        infos.inRange = true;
        //LOG_F(INFO, "In range.");
        if(onPoint){
            infos.isArrived = true;
            LOG_F(INFO, "On point.");
            return;
        }
    }

    if(!infos.isArrived)
    {
        //LOG_F(INFO, "Not on the point yet.");
        calculate_vector(px, py, pz, target_coordinates[0], target_coordinates[0], target_coordinates[0], infos.vectors.vx, infos.vectors.vy, infos.vectors.vz);
        calculate_rotation(blc_test[7], target_coordinates[4], infos.vectors.vr);

        
        if(px < bx){ 
            // Ici faire en sorte que la donnée soit supérieure (entre ]0;1])
            infos.ratiox = 1;
        }
        else{
            // Ici faire en sorte que la donnée soit supérieure (entre [-1;0[)
            infos.ratiox = -1;
        }

        if(py < by){ 
            infos.ratioy = 1;
        }
        else{
            infos.ratioy = -1;
        }

        if(pz < bz){ 
            infos.ratioz = 1;
        }
        else{
            infos.ratioz = -1;
        }

        LOG_F(INFO, "Informations 4 (Translated vector): vx: %f | vy: %f | vz: %f | vr: %f", infos.vectors.vx, infos.vectors.vy, infos.vectors.vz, infos.vectors.vr);
    }
}

void convert_polar_to_cartesian(float lat, float lon, float alt, float& x, float& y, float& z)
{
    x = EARTH_RADIUS * cos(lat) * cos(lon);
    y = EARTH_RADIUS * cos(lat) * sin(lon);
    z = alt*1000;
}

/*void convert_cartesian_to_polar(float x, float y, float z, float r, float& lat, float& lon){
    lat = asin(z / r);
    lon = atan2(y, x);
}*/

void calculate_rotation(float hdg, float targeted_hdg, float& r)
{
    float theta = abs(hdg-targeted_hdg);

    if(theta<180)
    {
        r=-1;
    }
    else{
        r=1;
    }
    
}

void calculate_vector(float x_drone, float y_drone, float z_drone, float x_target, float y_target, float z_target, float& x, float& y, float& z)
{
    x= x_drone-x_target;
    y= y_drone-y_target;
    z= z_drone-z_target;
}

