#include "Informations.h"

Informations::Informations(){
	this->ratioX = 0;
	this->ratioY = 0;
	this->ratioZ = 0;
	this->ratioR = 0;
	this->inRange = false;
	this->isArrived = false;
}
Informations::Informations(float ratioX, float ratioY, float ratioZ, float ratioR){
	this->ratioX = ratioX;
	this->ratioY = ratioY;
	this->ratioZ = ratioZ;
	this->ratioR = ratioR;
	this->inRange = false;
	this->isArrived = false;
}