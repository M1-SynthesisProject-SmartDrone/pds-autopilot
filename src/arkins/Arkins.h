#ifndef __ARKINS_H__
#define __ARKINS_H__

#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <chrono>
#include <stdlib.h>
#include <loguru/loguru.hpp>

#include "../data/Coordinates.h"
#include "../data/Informations.h"
#include "../data/Vector.h"

using namespace std;

#define EARTH_RADIUS 6371
#define MATH_PI 3.141592
#define RANGE 10000
#define UNITARY_VECTOR 0.5
#define ATTRACTION_POINTS_SIZE 3
#define REPULSION_RADIUS 100

class Arkins
{
private:
	Coordinates droneCoordinates;
	std::vector<Coordinates> attractionPoints;
	std::vector<Coordinates> repulsionPoints;
	std::vector<Coordinates> tangentialPoints;
	Informations infos;


public:
	Arkins();
	Arkins(Coordinates droneCoordinates, std::vector<Coordinates> attractionPoints, std::vector<Coordinates> repulsionPoints,
		std::vector<Coordinates> tangentialPoints);

	Informations& getInfos();

	void process(Coordinates droneCoordinates); // Process all the environment to calculate the vector to follow
	void deleteAttractivePoint(); // Delete an attractive point from the list [useful after visiting one of the points]
	void resetAttractivePoints(std::vector<Coordinates> attractionPoints); // Reset all the attractive points [useful after visiting every points]

private:
	void calculate_rotation(float hdg, float targeted_hdg, float& r); // Calculate how much rotation to do from a rotation to another
	void calculate_vector(Coordinates droneCoordinates, Coordinates targetCoordinates, float& x, float& y, float& z); // Calculate the vector between two positions
	bool isInRepulsionRadius(Coordinates& droneCoordinates, Coordinates& repulsionPoint); // Check if the drone is within the repulsion radius of given point
	void calculate_dist_between_points(Coordinates droneCoordinates, Coordinates& attractionPoint); // Calculate the distance between two points
	void calculate_coefficient_attraction(std::vector<Coordinates>& vector, float maxDistance); // Calculate the attraction coefficient from a list of attractive points
	void calculate_ratios(Coordinates droneCoordinates, Informations& infos, Coordinates attractivePoint); // Calculate the ratios and filling the structure to return
	Coordinates findMax(std::vector<Coordinates> vector);
	Coordinates calculate_barycenter(std::vector<Coordinates> vector);
};
#endif // __ARKINS_H__