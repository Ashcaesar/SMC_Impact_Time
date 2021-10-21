#include<stdlib.h>
#include<iostream>
#include<vector>

#include"quadrotor.h"

extern const double g;
extern const double dt;
extern const double PI;
extern const double mu;
extern const double rho;
extern const double k1;
extern const double k2_bar;
extern const double alpha1;
extern const double alpha2;

Quadrotor::Quadrotor(void) {};

Quadrotor::~Quadrotor() {};

bool Quadrotor::init(const std::vector<double>& para) {
	if (para.size() != 6) {
		cout << "number of quadrotor's parameter is wrong, initial failed!" << endl;
		return false;
	}
	POS.x = para[0];
	POS.y = para[1];
	VELOCITY = para[2];
	GAMMA = para[3] * PI / 180;
	T_d = para[4];
	THETA_d = para[5] * PI / 180;
	nu = 0;
	temp_dnu = 0;

	ACCELERATION_Mn = ACCELERATION_Mt = 0;
	ACC.x = ACCELERATION_Mn * cos(GAMMA + PI / 2) + ACCELERATION_Mt * cos(GAMMA);
	ACC.y = ACCELERATION_Mn * sin(GAMMA + PI / 2) + ACCELERATION_Mt * sin(GAMMA);
	VEL.x = VELOCITY * cos(GAMMA);
	VEL.y = VELOCITY * sin(GAMMA);
	updateRange();
	updateTheta();
	updateTheta_m();
	updateE_theta();
	updateTgo();
	return true;
}

/*************************************************************/
//public get function
/************************************************************/

double Quadrotor::dnu(void) {
	return -alpha2 * pow(abs(S2), 1 / 3)*sign(S2);
}

double Quadrotor::dnu(const double& t) {
	double s = t + RANGE / (VELOCITY*cos(THETA_M)) - T_d;
	return -alpha2 * pow(abs(s), 1 / 3)*sign(s);
}

double Quadrotor::dk2(double t) {
	return k2_bar * abs(S1);
}

int Quadrotor::getIndex(void) const {
	return INDEX;
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

pair<double,double> Quadrotor::getAccleration(void) const {
	return { ACCELERATION_Mn,ACCELERATION_Mt };
}

double Quadrotor::getTgo(void) const {
	return Tgo;
}

double Quadrotor::getTd(void) const {
	return T_d;
}

/*************************************************************/
//public set function
/************************************************************/

void Quadrotor::setIndex(const int& para) {
	INDEX = para;
}

void Quadrotor::setVelocity(const double& para) {
	VELOCITY = para;
}

void Quadrotor::setTarget(const Target& para) {
	target = para;
}

void Quadrotor::setTd(const double& para) {
	T_d = para;
}

void Quadrotor::updateRange(void) {
	RANGE = sqrt((target.getPos() - POS) * (target.getPos() - POS));
}

void Quadrotor::updateGamma(void) {
	GAMMA = atan2(VEL.y, VEL.x);
}

void Quadrotor::updateTheta(void) {
	THETA = atan2(target.getPos().y - POS.y, target.getPos().x - POS.x);
}

void Quadrotor::updateTheta_m(void) {
	THETA_M = GAMMA - THETA;
}

void Quadrotor::updateE_theta(void) {
	e_THETA = THETA - THETA_d;
}

void Quadrotor::updateTgo(void) {
	Tgo = RANGE / (VELOCITY*cos(THETA_M));
}

void Quadrotor::updateS(const double& t) {
	updateTgo();
	updateE_theta();
	S1 = mu * pow(abs(e_THETA), rho)*sign(e_THETA) - VELOCITY * sin(THETA_M) / RANGE;
	S2 = t + Tgo - T_d;
}

void Quadrotor::updateAcc(vector<Quadrotor>& quad, const int& num, const double& t) {
	updateS(t);
	/**/double k2 = 0.7;/**/
	nu += (temp_dnu + dnu()) * dt / 2;
	temp_dnu = dnu();
	/*double k2 = integral_k2(0, t, 0.01);/**/
	double U1 = -phi(e_THETA)*VELOCITY*sin(THETA_M) / RANGE + k1 * pow(abs(S1), 0.5)*sign(S1) + k2 * sign(S1);
	double U2 = alpha1 * pow(abs(S2), 2 / 3)*sign(S2) - nu;
	ACCELERATION_Mt = -pow(VELOCITY*sin(THETA_M), 2)*cos(THETA_M) / RANGE + RANGE * sin(THETA_M)*U1 + pow(VELOCITY*cos(THETA_M), 2) / RANGE * cos(THETA_M)*U2;
	ACCELERATION_Mn = -pow(VELOCITY, 2)*sin(THETA_M)*(1 + pow(cos(THETA_M), 2)) / RANGE + RANGE * cos(THETA_M)*U1 - pow(VELOCITY*cos(THETA_M), 2) / RANGE * sin(THETA_M)*U2;

	ACCELERATION_Mt = abs(ACCELERATION_Mt) > 100 ? 100 * sign(ACCELERATION_Mt) : ACCELERATION_Mt;
	ACCELERATION_Mn = abs(ACCELERATION_Mn) > 200 ? 200 * sign(ACCELERATION_Mn) : ACCELERATION_Mn;
	
	ACC.x = ACCELERATION_Mn * cos(GAMMA + PI / 2) + ACCELERATION_Mt * cos(GAMMA);
	ACC.y = ACCELERATION_Mn * sin(GAMMA + PI / 2) + ACCELERATION_Mt * sin(GAMMA);
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

double Quadrotor::integral_Nu(const double& lower, const double& upper, const double& step) {
	double ans = 0;
	double value1 = dnu(lower), value2, s;
	for (double i = lower; i < upper; i += step) {
		value2 = dnu(i+step);
		s = (value1 + value2)*step / 2;
		value1 = value2;
		ans += s;
	}
	return ans;
}

double Quadrotor::integral_k2(const double& lower, const double& upper, const double& step) {
	double ans = 0.03;
	double value1 = dk2(lower), value2, s;
	for (double i = lower; i < upper; i += step) {
		value2 = dk2(i + step);
		s = (value1 + value2)*step / 2;
		value1 = value2;
		ans += s;
	}
	return ans;
}

double Quadrotor::sign(const double& x) {
	if (x > 0) return 1.0;
	else if (x < 0) return -1.0;
	return 0;
}

double Quadrotor::phi(const double& x) {
	if (x == 0 && S1 != 0) {
		return 0;
	}
	else {
		return mu * rho*pow(abs(x), rho - 1);
	}
}

double Quadrotor::sgmf(const double& x) {
	double alpha = 20;
	return 2 / (1 + exp(-alpha * x)) - 1;
}
