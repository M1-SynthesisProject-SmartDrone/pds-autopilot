#include "arkins/Arkins.h"

#include <loguru/loguru.hpp>

int main(int argc, char* argv[]){
	loguru::init(argc, argv);

	////////////////////////////////// START MOCKS //////////////////////////////////
	Coordinates droneCoordinates;
	std::vector<Coordinates> attractivePoints;
	std::vector<Coordinates> repulsivePoints;
	for (int index = 0; index < ATTRACTION_POINTS_SIZE; index++)
	{
		for (int repet = 0; repet < 2; repet++)
		{
			Coordinates point;
			point.x = (rand() % (int) (droneCoordinates.x + 10) + droneCoordinates.x);
			point.y = (rand() % (int) (droneCoordinates.y + 10) + droneCoordinates.y);
			point.z = (rand() % (int) (droneCoordinates.z + 10) + droneCoordinates.z);
			point.rotation = (rand() % (int) (droneCoordinates.rotation + 10) + droneCoordinates.rotation);
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