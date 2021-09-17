#include<stdlib.h>
#include<iostream>
#include<vector>

#include "quadrotor.h"

extern const double K;
extern const double M;
extern const double N;
extern const double P;
extern const double Eps_1;
extern const double Eps_2;
extern const double g;
extern const double PI;
extern const double dt;
extern const double Min_Distance;

Quadrotor::Quadrotor(void) {};

Quadrotor::~Quadrotor() {};

bool Quadrotor::init(const std::vector<double> para) {
	if (para.size() != 5) {
		std::cout << "number of quadrotor's parameter is wrong, initial failed!" << std::endl;
		return false;
	}
	POS.x = para[0];
	POS.y = para[1];
	VELOCITY = para[2];
	ACCELERATION = para[3];
	GAMMA = para[4] * PI / 180;

	ACC.x = ACCELERATION * cos(GAMMA + PI / 2);
	ACC.y = ACCELERATION * sin(GAMMA + PI / 2);
	VEL.x = VELOCITY * cos(GAMMA);
	VEL.y = VELOCITY * sin(GAMMA);
	RANGE = sqrt((target.getPos() - POS)*(target.getPos() - POS));
	THETA = atan2(target.getPos().y - POS.y, target.getPos().x - POS.x);
	THETA_M = GAMMA - THETA;
	updateTgo();
	updateS(0);
	Timp = getTgo();
	return true;
}

/*************************************************************/
//public get function
/************************************************************/

double Quadrotor::getTimp(void) const {
	return Timp;
}

double Quadrotor::getTgo(void) const {
	return Tgo;
}

double Quadrotor::getRange(void) const {
	return sqrt((target.getPos() - POS) * (target.getPos() - POS));
}

coordinate Quadrotor::getPos(void) const{
	return POS;
}

coordinate Quadrotor::getVel(void) const {
	return VEL;
}

double Quadrotor::getVelocity(void) const {
	return VELOCITY;
}

coordinate Quadrotor::getAcc(void) const {
	return ACC;
}

double Quadrotor::getAccleration(void) const {
	return ACCELERATION;
}

/*************************************************************/
//public set function
/************************************************************/

void Quadrotor::setIndex(const int& x) {
	INDEX = x;
}

void Quadrotor::setVelocity(const double& para) {
	VELOCITY = para;
}

void Quadrotor::setAccleration(const double& para) {
	ACCELERATION = para;
	ACC.x = ACCELERATION * cos(GAMMA + PI / 2);
	ACC.y = ACCELERATION * sin(GAMMA + PI / 2);
}

void Quadrotor::setTimp(const double& t) {
	Timp = t;
}

void Quadrotor::setTarget(const Target& t) {
	target = t;
}

void Quadrotor::updateRange(void) {
	RANGE = sqrt((target.getTP() - POS) * (target.getTP() - POS));
}

void Quadrotor::updateGamma(void) {
	GAMMA = atan2(VEL.y, VEL.x);
}

void Quadrotor::updateTheta(void) {
	THETA = atan2(target.getTP().y - POS.y, target.getTP().x - POS.x);
}

void Quadrotor::updateTheta_m(void) {
	THETA_M = GAMMA - THETA;
}

void Quadrotor::updateTgo(void) {
	Tgo = RANGE / VELOCITY * (1 + THETA_M * THETA_M / (4 * N - 2));
}

void Quadrotor::updateS(const double& t) {
	S = Tgo - (Timp - t);
}

void Quadrotor::updateAcc(Quadrotor* quad, int num) {
	//滑模面控制加速度
	ACC_EQ = ((2 * N - 1)*(cos(THETA_M) - 1) / THETA_M + THETA_M * cos(THETA_M) / 2 - sin(THETA_M))*VELOCITY*VELOCITY / RANGE;
	ACC_MCON = -H(THETA_M) / THETA_M * K*S;
	ACC_SW = -M * (P*sgmf(THETA_M) + 1) * sgmf(S);
	ACCELERATION = ACC_EQ + ACC_MCON + ACC_SW;
	if (abs(ACCELERATION) >  g) {
		ACCELERATION = ACCELERATION > 0 ? g : -g;
	}

	//斥力惩罚
	ACC_RUPLSION.x = ACC_RUPLSION.y = 0;
	for (int i = 0; i < num; i++) {
		if (i != INDEX) {
			double distance = sqrt((quad[i].POS - POS) * (quad[i].POS - POS));
			if (distance < Min_Distance) {
				ACC_RUPLSION = ACC_RUPLSION + (POS - quad[i].POS)*(1.5*(12.0 - distance) / distance + 1e-6);
			}
		}
	}

	ACC.x = ACC_RUPLSION.x + ACCELERATION * cos(GAMMA + PI / 2);
	ACC.y = ACC_RUPLSION.y + ACCELERATION * sin(GAMMA + PI / 2);
}

void Quadrotor::updateState(void) {
	VEL = VEL + ACC * dt;
	VELOCITY = sqrt(VEL * VEL);
	updateGamma();
	//GAMMA = atan2(VELY, VELX);
	POS = POS + VEL * dt;
	updateTheta();
	//THETA = atan2(target.getTP()[1] - POSY, target.getTP()[0] - POSX);
	updateRange();
	//RANGE = sqrt(pow(target.getTP()[0] - POSX, 2) + pow(target.getTP()[1] - POSY, 2));
	updateTheta_m();
	//THETA_M = GAMMA - THETA;
}

double Quadrotor::sign(const double& x) {
	if (x > 0) return 1.0;
	else if (x < 0) return -1.0;
	return 0;
}

double Quadrotor::H(const double& x) {
	double ans;
	if (abs(x) < Eps_1) {
		ans = abs(x);
	}
	else if (abs(x) >= Eps_1 && abs(x) <= Eps_2) {
		ans = (1 - Eps_1) / (1 - Eps_2)*abs(x) + Eps_1 * (Eps_2 - 1) / (Eps_2 - Eps_1);
	}
	else ans = 1.0;
	return ans;
}

double Quadrotor::sgmf(const double& x) {
	double alpha = 20;
	return 2 / (1 + exp(-alpha * x)) - 1;
}
