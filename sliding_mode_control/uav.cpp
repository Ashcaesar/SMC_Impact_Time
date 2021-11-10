#include<stdlib.h>
#include<iostream>
#include<vector>

#include"function.h"
#include"uav.h"

extern const double g;
extern const double dt;
extern const double PI;
extern const double mu;
extern const double rho;
extern const double k1;
extern const double k2_0;
extern const double k2_bar;
extern const double alpha1;
extern const double alpha2;

extern const double K;
extern const double M;
extern const double N;
extern const double P;
extern const double Eps_1;
extern const double Eps_2;

Uav::Uav(void) {};

Uav::~Uav() {};

bool Uav::init(const std::vector<double>& para) {
	if (para.size() != 6) {
		cout << "number of quadrotor's parameter is wrong, initial failed!" << endl;
		return false;
	}
	POS.x = para[0];
	POS.y = para[1];
	VELOCITY = para[2];
	GAMMA = para[3] * PI / 180;
	T_d_ = para[4];
	THETA_d_ = para[5] * PI / 180;
	nu = 0;
	k2 = k2_0;

	ACCELERATION_Mn_ = ACCELERATION_Mt_ = 0;
	ACC.x = ACCELERATION_Mn_ * cos(GAMMA + PI / 2) + ACCELERATION_Mt_ * cos(GAMMA);
	ACC.y = ACCELERATION_Mn_ * sin(GAMMA + PI / 2) + ACCELERATION_Mt_ * sin(GAMMA);
	VEL.x = VELOCITY * cos(GAMMA);
	VEL.y = VELOCITY * sin(GAMMA);
	updateRange();
	updateTheta();
	updateTheta_m();
	updateE_theta();
	bool mode = true;
	updateT_go_(mode);
	return true;
}

/*************************************************************/
//public get function
/************************************************************/

int Uav::getIndex(void) const {
	return INDEX;
}

double Uav::getRange(void) const {
	return sqrt((TARGET.getPos() - POS) * (TARGET.getPos() - POS));
}

coordinate Uav::getPos(void) const{
	return POS;
}

coordinate Uav::getVel(void) const {
	return VEL;
}

double Uav::getVelocity(void) const {
	return VELOCITY;
}

coordinate Uav::getAcc(void) const {
	return ACC;
}

pair<double,double> Uav::getAccleration(void) const {
	return { ACCELERATION_Mn_,ACCELERATION_Mt_ };
}

double Uav::getT_go_(void) const {
	return T_go_;
}

double Uav::getT_d_(void) const {
	return T_d_;
}

double Uav::getGamma(void) const {
	return GAMMA;
}

double Uav::getTHETA(void) const {
	return THETA;
}

double Uav::getTHETA_d_(void) const {
	return THETA_d_;
}

double Uav::getE_THETA_(void) const {
	return E_THETA_;
}

/*************************************************************/
//public set function
/************************************************************/

void Uav::resetNu(void) {
	nu = 0;
}

void Uav::resetK2(void) {
	k2 = k2_0;
}

void Uav::setIndex(const int& index) {
	INDEX = index;
}

void Uav::setVelocity(const double& velocity) {
	VELOCITY = velocity;
}

void Uav::setTarget(const Target& target) {
	TARGET = target;
}

void Uav::setT_d_(const double& td) {
	T_d_ = td;
}

void Uav::updateRange(void) {
	RANGE = sqrt((TARGET.getPos() - POS) * (TARGET.getPos() - POS));
}

void Uav::updateGamma(void) {
	GAMMA = atan2(VEL.y, VEL.x);
}

void Uav::updateTheta(void) {
	THETA = atan2(TARGET.getPos().y - POS.y, TARGET.getPos().x - POS.x);
}

void Uav::updateTheta_m(void) {
	THETA_m_ = GAMMA - THETA;
}

void Uav::updateE_theta(void) {
	E_THETA_ = THETA - THETA_d_;
}

void Uav::updateT_go_(bool& mode) {
	T_go_ = mode == true ? RANGE / (VELOCITY*cos(THETA_m_)) : RANGE / VELOCITY * (1 + THETA_m_ * THETA_m_ / (4 * N - 2));
}

void Uav::updateS(const double& t, bool& mode) {
	updateT_go_(mode);
	//集结模式
	if (mode) {
		updateE_theta();
		S1 = mu * pow(abs(E_THETA_), rho)*sign(E_THETA_) - VELOCITY * sin(THETA_m_) / RANGE;
		S2 = t + T_go_ - T_d_;
	}
	//编队模式
	else {
		S = t + T_go_ - T_d_;
	}
}

void Uav::updateAcc(vector<Uav>& uavs, const int& num, const double& t, bool& mode) {
	updateS(t, mode);
	//集结模式
	if (mode) {
		k2 = integral(dk2, S1, k2, dt);
		nu = integral(dnu, alpha2, S2, nu, dt);
		double U1 = -phi(E_THETA_)*VELOCITY*sin(THETA_m_) / RANGE + k1 * pow(abs(S1), 0.5)*sign(S1) + k2 * sign(S1);
		double U2 = alpha1 * pow(abs(S2), 2 / 3)*sign(S2) - nu;
		ACCELERATION_Mt_ = -pow(VELOCITY*sin(THETA_m_), 2)*cos(THETA_m_) / RANGE + RANGE * sin(THETA_m_)*U1 + pow(VELOCITY*cos(THETA_m_), 2) / RANGE * cos(THETA_m_)*U2;
		ACCELERATION_Mn_ = -pow(VELOCITY, 2)*sin(THETA_m_)*(1 + pow(cos(THETA_m_), 2)) / RANGE + RANGE * cos(THETA_m_)*U1 - pow(VELOCITY*cos(THETA_m_), 2) / RANGE * sin(THETA_m_)*U2;

		ACCELERATION_Mt_ = abs(ACCELERATION_Mt_) > 100 ? 100 * sign(ACCELERATION_Mt_) : ACCELERATION_Mt_;
		ACCELERATION_Mn_ = abs(ACCELERATION_Mn_) > 200 ? 200 * sign(ACCELERATION_Mn_) : ACCELERATION_Mn_;
	
		ACC.x = ACCELERATION_Mn_ * cos(GAMMA + PI / 2) + ACCELERATION_Mt_ * cos(GAMMA);
		ACC.y = ACCELERATION_Mn_ * sin(GAMMA + PI / 2) + ACCELERATION_Mt_ * sin(GAMMA);
	}
	//编队模式
	else {
		ACC_EQ = ((2 * N - 1)*(cos(THETA_m_) - 1) / THETA_m_ + THETA_m_ * cos(THETA_m_) / 2 - sin(THETA_m_))*VELOCITY*VELOCITY / RANGE;
		ACC_MCON = ACC_MCON = -H(THETA_m_) / THETA_m_ * K*S;
		ACC_SW = -M * (P*sgmf(THETA_m_) + 1) * sgmf(S);
		
		ACCELERATION = ACC_EQ + ACC_MCON + ACC_SW;
		if (abs(ACCELERATION) > g) {
			ACCELERATION = ACCELERATION > 0 ? g : -g;
		}
		
		ACC.x = ACCELERATION * cos(GAMMA + PI / 2);
		ACC.y = ACCELERATION * sin(GAMMA + PI / 2);
	}
}

void Uav::updateState(void) {
	VEL = VEL + ACC * dt;
	VELOCITY = sqrt(VEL * VEL);
	updateGamma();
	POS = POS + VEL * dt;
	updateTheta();
	updateRange();
	updateTheta_m();
}

double Uav::integral(double(*f)(const double&, const double&), const double& x1, const double& x2, double& prev, const double& step) {
	double value = f(x1, x2);
	double ans = (prev + value)*step / 2;
	return ans;
}

double Uav::integral(double(*f)(const double&), const double& x, double& prev, const double& step) {
	double value = f(x);
	double ans = (prev + value)*step / 2;
	return ans;
}

double Uav::phi(const double& x) {
	if (x == 0 && S1 != 0) {
		return 0;
	}
	else {
		return mu * rho*pow(abs(x), rho - 1);
	}
}

double Uav::sgmf(const double& x) {
	double alpha = 20;
	return 2 / (1 + exp(-alpha * x)) - 1;
}

double Uav::H(const double& x) {
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