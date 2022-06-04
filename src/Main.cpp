#include "arkins/Arkins.h"

#include <time.h>

#include <loguru/loguru.hpp>

Coordinates createRandomPoint();

int main(int argc, char* argv[]){
	loguru::init(argc, argv);
	srand(time(NULL));

	////////////////////////////////// START MOCKS //////////////////////////////////
	Coordinates droneCoordinates;
	std::vector<Coordinates> attractivePoints;
	std::vector<Coordinates> repulsivePoints;
	for (int index = 0; index < ATTRACTION_POINTS_SIZE; index++)
	{
		for (int repet = 0; repet < 2; repet++)
		{
			Coordinates point = createRandomPoint();
			if (repet == 1)
			{
				repulsivePoints.push_back(point);
				// LOG_F(INFO, "Information 0 (RepPoint n°%d) : alt = %f | lat = %f | long = %f | rot = %f\n", index + 1, repulsivePoints.at(index).getAltitude(),
				//                 repulsivePoints.at(index).getLatitude(), repulsivePoints.at(index).getLongitude(), repulsivePoints.at(index).getRotation());
			}
			else
			{
				attractivePoints.push_back(point);
				// LOG_F(INFO, "Information 0 (AttPoint n°%d) : alt = %f | lat = %f | long = %f | rot = %f", index + 1, attractivePoints.at(index).getAltitude(),
				//                 attractivePoints.at(index).getLatitude(), attractivePoints.at(index).getLongitude(), attractivePoints.at(index).getRotation());
			}
		}
	}
	//LOG_F(INFO, "AttPointsVectorSize : %ld | RepPointsVectorSize : %ld", attractivePoints.size(), repulsivePoints.size());
	////////////////////////////////// END MOCKS //////////////////////////////////

	Arkins arkins = Arkins(attractivePoints, repulsivePoints, attractivePoints);
	
	//TWO POINTS
	arkins.process(droneCoordinates);
	arkins.deleteAttractivePoint();
	//ONE POINTS
	arkins.process(droneCoordinates);
	arkins.deleteAttractivePoint();
	//ZERO POINTS
	arkins.process(droneCoordinates);
	arkins.deleteAttractivePoint();
}

float randomFloat(float min, float max)
{
	return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
}

Coordinates createRandomPoint()
{
	Coordinates c;

	c.x = randomFloat(-10.0f, 10.0f);
	c.y = randomFloat(-10.0f, 10.0f);
	c.z = randomFloat(-10.0f, 10.0f);
	c.rotation = randomFloat(-10.0f, 10.0f);
	return c;
}