#include "Arkins.h"

Arkins::Arkins()
{

}

Arkins::Arkins(std::vector<Coordinates> attractionPoints, std::vector<Coordinates> repulsionPoints,
	std::vector<Coordinates> tangentialPoints) : attractionPoints(attractionPoints),
	repulsionPoints(repulsionPoints), tangentialPoints(tangentialPoints)
{}

Informations& Arkins::getInfos()
{
	return infos;
}

void Arkins::process(Coordinates& droneCoordinates)
{
	for (int attPoints = 0; attPoints < attractionPoints.size(); attPoints++)
	{
		calculate_dist_between_points(droneCoordinates, attractionPoints.at(attPoints));
	}

	Coordinates maxPoint = findMax(attractionPoints);

	calculate_coefficient_attraction(attractionPoints, maxPoint.distance_to_drone);
	Coordinates barycenter = calculate_barycenter(attractionPoints);
	calculate_ratios(droneCoordinates, infos, barycenter);
}

void Arkins::deleteAttractivePoint()
{
	attractionPoints.erase(attractionPoints.begin());
}

void Arkins::resetAttractivePoints(std::vector<Coordinates> attractionPoints)
{
	this->attractionPoints = attractionPoints;
}

Coordinates Arkins::findMax(std::vector<Coordinates>& vector)
{
	int max = 0;
	Coordinates maxPoint;
	for (int index = 0; index < vector.size(); index++)
	{
		Coordinates point = vector.at(index);
		if (max < point.distance_to_drone)
		{
			max = point.distance_to_drone;
			maxPoint = point;
		}
	}
	return maxPoint;
}

Coordinates Arkins::calculate_barycenter(std::vector<Coordinates>& vector)
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

void Arkins::calculate_dist_between_points(Coordinates& droneCoordinates, Coordinates& attractionPoint)
{
	float dist = abs(sqrt(pow((droneCoordinates.x - attractionPoint.x), 2) + pow((droneCoordinates.y - attractionPoint.y), 2) + pow((droneCoordinates.z - attractionPoint.z), 2)));
	attractionPoint.distance_to_drone = dist;
}

void Arkins::calculate_coefficient_attraction(std::vector<Coordinates>& vector, float maxDistance)
{
	float coef = 0;
	for (auto& coordinate : vector)
	{
		coef = 1 - coordinate.distance_to_drone / maxDistance;
		coordinate.attraction = coef; //+ on est proche du point d'attraction, + le coefficient sera grand [d'où le 1 - rapport distance/maxDistance]
	}
}


void Arkins::calculate_vector(Coordinates& droneCoordinates, Coordinates& targetCoordinates, float& x, float& y, float& z)
{
	x = droneCoordinates.x - targetCoordinates.x;
	y = droneCoordinates.y - targetCoordinates.y;
	z = droneCoordinates.z - droneCoordinates.z;
}

bool Arkins::isInRepulsionRadius(Coordinates& droneCoordinates, Coordinates& repulsionPoint)
{
	if (droneCoordinates.x <= repulsionPoint.x + REPULSION_RADIUS && droneCoordinates.x >= repulsionPoint.x - REPULSION_RADIUS)
	{
		if (droneCoordinates.y <= repulsionPoint.y + REPULSION_RADIUS && droneCoordinates.z >= repulsionPoint.z - REPULSION_RADIUS)
		{
			return true;
		}
	}
	return false;
}

void Arkins::calculate_rotation(float hdg, float targeted_hdg, float& r)
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

void Arkins::calculate_ratios(Coordinates& droneCoordinates, Informations& infos, Coordinates& attractivePoint)
{
	/*
		Ce qui sera retourné
		infos : x, y, z, r (entre -1 et 1), inRange [disant si le drone est a portée] et isArrived [disant si le drone est bien arrivé]
	*/
	// Coordonnées cartésiennes du drone
	float px = droneCoordinates.x;
	float py = droneCoordinates.y;
	float pz = droneCoordinates.z;

	// Coordonées cartésiennes du point de destination
	float bx = attractivePoint.x;
	float by = attractivePoint.y;
	float bz = attractivePoint.z;

	LOG_F(INFO, "Informations 2 (Drone cartesian): px: %f | py: %f | pz: %f", px, py, pz);
	LOG_F(INFO, "Informations 3 (But) : bx: %f | by: %f | bz: %f", bx, by, bz);

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
		calculate_vector(droneCoordinates, attractivePoint, infos.vector.vx, infos.vector.vy, infos.vector.vz);
		calculate_rotation(droneCoordinates.rotation, attractivePoint.rotation, infos.vector.vr);

		//LOG_F(INFO, "Informations 4 (Translated vector): vx: %f | vy: %f | vz: %f | vr: %f", infos.vector.vx, infos.vector.vy, infos.vector.vz, infos.vector.vr);

		if (px < bx)
		{
			// Ici faire en sorte que la donnée soit supérieure (entre ]0;1])
			infos.ratioX = 1;
		}
		else
		{
			// Ici faire en sorte que la donnée soit supérieure (entre [-1;0[)
			infos.ratioX = -1;
		}

		if (py < by)
		{
			infos.ratioY = 1;
		}
		else
		{
			infos.ratioY = -1;
		}

		if (pz < bz)
		{
			infos.ratioZ = 1;
		}
		else
		{
			infos.ratioZ = -1;
		}

		LOG_F(INFO, "Informations 5 (info struct): rx = %f | ry = %f | rz = %f | rr = %f | inRange = %d | isArrived = %d", infos.ratioX, infos.ratioY, infos.ratioZ, infos.ratioR, infos.inRange, infos.isArrived);
	}
}