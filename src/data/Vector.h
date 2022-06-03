#ifndef __VECTOR_H__
#define __VECTOR_H__

struct Vector {
	float vx = 0;
	float vy = 0;
	float vz = 0;
	float vr = 0;
	Vector();
	Vector(float vx, float vy, float vz, float vr);
};
#endif // __VECTOR_H__