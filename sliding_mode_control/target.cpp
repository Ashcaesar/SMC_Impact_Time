#include<stdlib.h>
#include<iostream>
#include<vector>

#include "target.h"

extern const double dt;
extern const double PI;

Target::Target(void) {};
Target::~Target() {};

bool Target::init(std::vector<double> para) {
	if (para.size() != 4) {
		std::cout << "number of target's parameter is wrong, initial failed!" << std::endl;
		return false;
	}
	POS.x = para[0];
	POS.y = para[1];
	VELOCITY = para[2];
	GAMMA = para[3] * PI / 180;

	VEL.x = VELOCITY * cos(GAMMA);
	VEL.y = VELOCITY * sin(GAMMA);
	return true;
}

coordinate Target::getPos(void) const {
	return POS;
}

coordinate Target::getVel(void) const {
	return VEL;
}

double Target::getVelocity(void) const {
	return VELOCITY;
}

coordinate Target::getTP(void) const {
	return TP;
}

double Target::getGamma(void) const {
	return GAMMA;
}
void Target::setGamma(const double& para) {
	GAMMA = para;
}

void Target::setPos(const coordinate& para) {
	POS = para;
}

void Target::setVel(const coordinate& para) {
	VEL = para;
}

void Target::setVelocity(const double& para) {
	VELOCITY = para;
}

void Target::updateState(void) {
	VEL.x = VELOCITY * cos(GAMMA);
	VEL.y = VELOCITY * sin(GAMMA);
	POS = POS + VEL * dt;
}

void Target::updateTP(const double& tgo) {
	TP.x = POS.x + VELOCITY * cos(GAMMA)*tgo;
	TP.y = POS.y + VELOCITY * cos(GAMMA)*tgo;
}