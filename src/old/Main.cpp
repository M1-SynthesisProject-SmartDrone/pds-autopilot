#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <loguru/loguru.hpp>
#include <math.h>
#include <chrono>
#include <stdlib.h>

using namespace std;

#define EARTH_RADIUS 6371
#define MATH_PI 3.141592
#define RANGE 10000
#define UNITARY_VECTOR 0.5
#define ATTRACTION_POINTS_SIZE 100000
#define REPULSION_RADIUS 100

struct Vector
{
	float vx = 0.0;
	float vy = 0.0;
	float vz = 0.0;
	float vr = 0.0;
};

struct Informations
{
	float ratiox = 0;
	float ratioy = 0;
	float ratioz = 0;
	float ratior = 0;
	bool inRange = false;
	bool isArrived = false;
	Vector vectors;
};

struct Coordinates
{
	float latitude = 0;
	float longitude = 0;
	float altitude = 0;
	float rotation = 0;
	float x = 0;
	float y = 0;
	float z = 0;
	float attraction = 0;		 // coefficient d'attraction
	float distance_to_point = 0; // distance entre point et drone
};

float blc_test[8] = {12.0333, 3.55456, 10200, 0, 0, 0, 0, 45.3}; //{lat/lon/alt/relative_alt/vx/vy/vz/hdg} mocked blc_channels
float command_motors[4];//{x,y,z,r}
float target_coordinates[4] = {
	2034.32,
	6352.65,
	5,
}; //{x,y,z,r} mocked

void convert_polar_to_cartesian(float lat, float lon, float alt, float &x, float &y, float &z);
void convert_polar_to_cartesian(Coordinates &point);
void convert_cartesian_to_polar(float x, float y, float z, float &lat, float &lon, float &alt);

void calculate_rotation(float hdg, float targeted_hdg, float &r);
void calculate_vector(float x_drone, float y_drone, float z_drone, float x_target, float y_target, float z_target, float &x, float &y, float &z);
bool isInRepulsionRadius(Coordinates &droneCoordinates, Coordinates &repulsionPoint);
void calculate_dist_between_points(Coordinates droneCoordinates, Coordinates &attractionPoint);
void calculate_coefficient_attraction(std::vector<Coordinates> &vector, float maxDistance);
void calculate_ratios(Coordinates droneCoordinates, Informations &infos, Coordinates attractivePoint);
Coordinates findMax(std::vector<Coordinates> vector);
Coordinates calculate_barycenter(std::vector<Coordinates> vector);

int main(int argc, char *argv[])
{
	srand(time(0));
	struct Informations infos;
	struct Coordinates droneCoordinates;
	droneCoordinates.altitude = blc_test[2];
	droneCoordinates.longitude = blc_test[1];
	droneCoordinates.latitude = blc_test[0];
	droneCoordinates.rotation = blc_test[3];
	/*LOG_F(INFO, "Information 0 (Position) : alt = %f | lat = %f | long = %f | rot = %f", droneCoordinates.altitude,
		  droneCoordinates.latitude, droneCoordinates.longitude, droneCoordinates.rotation);*/

	////////////////////////////////// START MOCKS //////////////////////////////////
	std::vector<Coordinates> attractivePoints;
	std::vector<Coordinates> repulsivePoints;
	for (int index = 0; index < ATTRACTION_POINTS_SIZE; index++)
	{
		for (int repet = 0; repet < 2; repet++)
		{
			Coordinates point;
			point.altitude = rand() % int(droneCoordinates.altitude + 10) + droneCoordinates.altitude;
			point.latitude = rand() % int(droneCoordinates.latitude + 10) + droneCoordinates.latitude;
			point.longitude = rand() % int(droneCoordinates.longitude + 10) + droneCoordinates.longitude;
			point.rotation = rand() % int(droneCoordinates.rotation + 10) + droneCoordinates.rotation;
			if (repet == 1)
			{
				repulsivePoints.push_back(point);
				// LOG_F(INFO, "Information 0 (RepPoint n°%d) : alt = %f | lat = %f | long = %f | rot = %f\n", index + 1, repulsivePoints.at(index).altitude,
				//                 repulsivePoints.at(index).latitude, repulsivePoints.at(index).longitude, repulsivePoints.at(index).rotation);
			}
			else
			{
				attractivePoints.push_back(point);
				// LOG_F(INFO, "Information 0 (AttPoint n°%d) : alt = %f | lat = %f | long = %f | rot = %f", index + 1, attractivePoints.at(index).altitude,
				//                 attractivePoints.at(index).latitude, attractivePoints.at(index).longitude, attractivePoints.at(index).rotation);
			}
		}
	}
	//LOG_F(INFO, "AttPointsVectorSize : %ld | RepPointsVectorSize : %ld", attractivePoints.size(), repulsivePoints.size());
	////////////////////////////////// END MOCKS //////////////////////////////////

	loguru::init(argc, argv);
	//LOG_F(INFO, "Start the test autopilot program");
	chrono::steady_clock::time_point start = chrono::steady_clock::now();

	/*convert_polar_to_cartesian(droneCoordinates.altitude, droneCoordinates.latitude, droneCoordinates.longitude,
							   droneCoordinates.x, droneCoordinates.y, droneCoordinates.z);*/
	convert_polar_to_cartesian(droneCoordinates);
	for (int attPoints = 0; attPoints < attractivePoints.size(); attPoints++)
	{
		calculate_dist_between_points(droneCoordinates, attractivePoints.at(attPoints));
	}
	
	Coordinates maxPoint = findMax(attractivePoints);

	calculate_coefficient_attraction(attractivePoints, maxPoint.distance_to_point);
	Coordinates barycenter = calculate_barycenter(attractivePoints);
	calculate_ratios(droneCoordinates, infos, barycenter);

	//LOG_F(INFO, "End of the program test autopilot");

	chrono::steady_clock::time_point end = chrono::steady_clock::now();
	auto timeElapsed = chrono::duration_cast<chrono::microseconds>(end - start).count();
	LOG_F(INFO, "Process done in %ld microseconds", timeElapsed);
	//LOG_F(INFO, "Informations 5 (info struct): rx = %f | ry = %f | rz = %f | rr = %f | inRange = %d | isArrived = %d", infos.ratiox, infos.ratioy, infos.ratioz, infos.ratior, infos.inRange, infos.isArrived);
	return EXIT_SUCCESS;
}

Coordinates findMax(std::vector<Coordinates> vector)
{
	int max = 0;
	Coordinates maxPoint;
	for (int index = 0; index < vector.size(); index++)
	{
		Coordinates point = vector.at(index);
		if (max < point.distance_to_point)
		{
			max = point.distance_to_point;
			maxPoint = point;
		}
	}
	return maxPoint;
}

void calculate_dist_between_points(Coordinates droneCoordinates, Coordinates &attractionPoint)
{
	convert_polar_to_cartesian(attractionPoint);

	float dist = abs(sqrt(pow((droneCoordinates.x - attractionPoint.x), 2) + pow((droneCoordinates.y - attractionPoint.y), 2) + pow((droneCoordinates.z - attractionPoint.z), 2)));
	attractionPoint.distance_to_point = dist;
}

void calculate_coefficient_attraction(std::vector<Coordinates> &vector, float maxDistance)
{
	float coef = 0;
	for (auto &coordinate : vector)
	{
		coef = 1 - coordinate.distance_to_point / maxDistance;
		coordinate.attraction = coef; //+ on est proche du point d'attraction, + le coefficient sera grand [d'où le 1 - rapport distance/maxDistance]
	}
}

Coordinates calculate_barycenter(std::vector<Coordinates> vector)
{
	Coordinates barycenter;
	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;
	float sum_coef = 0;
	for (Coordinates point : vector)
	{
		sum_x += point.attraction * point.x;
		sum_y += point.attraction * point.y;
		sum_z += point.attraction * point.z;
		sum_coef += point.attraction;
	}
	barycenter.x = sum_x / sum_coef;
	barycenter.y = sum_y / sum_coef;
	barycenter.z = sum_z / sum_coef;
	barycenter.attraction = 1;
	return barycenter;
}

void calculate_ratios(Coordinates droneCoordinates, Informations &infos, Coordinates attractivePoint)
{
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
	float bLatitude = attractivePoint.latitude;
	float bLongitude = attractivePoint.longitude;
	float bAltitude = attractivePoint.altitude;
	float bRotation = attractivePoint.rotation;

	// Coordonées cartésiennes du point de destination
	float bx;
	float by;
	float bz;

	convert_polar_to_cartesian(latitude, longitude, altitude, px, py, pz);
	convert_polar_to_cartesian(bLatitude, bLongitude, bAltitude, bx, by, bz);

	/*LOG_F(INFO, "Informations 1 (Drone polar): lat: %f | long: %f | alt: %f", latitude, longitude, altitude);
	LOG_F(INFO, "Informations 2 (Drone cartesian): px: %f | py: %f | pz: %f", px, py, pz);
	LOG_F(INFO, "Informations 3 (But) : bx: %f | by: %f | bz: %f", bx, by, bz);*/

	bool isInRange = ((px > bx - RANGE && px < bx + RANGE) && (py > by - RANGE && py < by + RANGE) && (pz > bz - RANGE && pz < bz + RANGE));

	bool onPoint = (px == bx) && (py == by) && (pz == bz);

	// On effectue une comparaison afin de savoir dans quelle direction on doit se diriger en x, y et z
	if (isInRange)
	{
		// On alerte qu'on est entré dans la sphère d'arrivée, les ratios sont définies par défaut à 0
		infos.inRange = true;
		// LOG_F(INFO, "In range.");
		if (onPoint)
		{
			infos.isArrived = true;
			//LOG_F(INFO, "On point.");
			return;
		}
	}

	if (!infos.isArrived)
	{
		// LOG_F(INFO, "Not on the point yet.");
		calculate_vector(px, py, pz, bx, by, bz, infos.vectors.vx, infos.vectors.vy, infos.vectors.vz);
		calculate_rotation(rotation, bRotation, infos.vectors.vr);

		//LOG_F(INFO, "Informations 4 (Translated vector): vx: %f | vy: %f | vz: %f | vr: %f", infos.vectors.vx, infos.vectors.vy, infos.vectors.vz, infos.vectors.vr);

		if (px < bx)
		{
			// Ici faire en sorte que la donnée soit supérieure (entre ]0;1])
			infos.ratiox = 1;
		}
		else
		{
			// Ici faire en sorte que la donnée soit supérieure (entre [-1;0[)
			infos.ratiox = -1;
		}

		if (py < by)
		{
			infos.ratioy = 1;
		}
		else
		{
			infos.ratioy = -1;
		}

		if (pz < bz)
		{
			infos.ratioz = 1;
		}
		else
		{
			infos.ratioz = -1;
		}

		//LOG_F(INFO, "Informations 5 (info struct): rx = %f | ry = %f | rz = %f | rr = %f | inRange = %d | isArrived = %d", infos.ratiox, infos.ratioy, infos.ratioz, infos.ratior, infos.inRange, infos.isArrived);
	}
}

void convert_polar_to_cartesian(float lat, float lon, float alt, float &x, float &y, float &z)
{
	x = EARTH_RADIUS * cos(lat) * cos(lon);
	y = EARTH_RADIUS * cos(lat) * sin(lon);
	z = alt * 1000;
}

void convert_polar_to_cartesian(Coordinates &point)
{
	point.x = EARTH_RADIUS * cos(point.latitude) * cos(point.longitude);
	point.y = EARTH_RADIUS * cos(point.latitude) * sin(point.longitude);
	point.z = point.altitude * 1000;
}

/*void convert_cartesian_to_polar(float x, float y, float z, float r, float& lat, float& lon){
	lat = asin(z / r);
	lon = atan2(y, x);
}*/

void calculate_rotation(float hdg, float targeted_hdg, float &r)
{
	float theta = abs(hdg - targeted_hdg);

	if (theta < 180)
	{
		r = -1;
	}
	else
	{
		r = 1;
	}
}

void calculate_vector(float x_drone, float y_drone, float z_drone, float x_target, float y_target, float z_target, float &x, float &y, float &z)
{
	x = x_drone - x_target;
	y = y_drone - y_target;
	z = z_drone - z_target;
}

bool isInRepulsionRadius(Coordinates &droneCoordinates, Coordinates &repulsionPoint){
	convert_polar_to_cartesian(droneCoordinates);
	convert_polar_to_cartesian(repulsionPoint);
	if(droneCoordinates.x <= repulsionPoint.x + REPULSION_RADIUS && droneCoordinates.x >= repulsionPoint.x - REPULSION_RADIUS){
		if(droneCoordinates.y <= repulsionPoint.y + REPULSION_RADIUS && droneCoordinates.z >= repulsionPoint.z - REPULSION_RADIUS){
				return true;
		}
	}
	return false;
}

float calculate_repulsion_coefficient(Coordinates &droneCoordinates, Coordinates &repulsionPoint){

}