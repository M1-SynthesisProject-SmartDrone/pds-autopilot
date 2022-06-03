#ifndef __COORDINATES_H__
#define __COORDINATES_H__

struct Coordinates {
		float x = 0;
		float y = 0;
		float z = 0;
		float rotation = 0; 
		float attraction = 0;		 // coefficient d'attraction
		float distance_to_drone = 0; // distance entre point et drone

		Coordinates();
		Coordinates(float x, float y, float z, float r);
};
#endif // __COORDINATES_H__