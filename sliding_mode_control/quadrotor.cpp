#include<stdlib.h>
#include<iostream>
#include<vector>

#include"function.h"
#include"quadrotor.h"

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
	updateTgo();
	return true;
}

/*************************************************************/
//public get function
/************************************************************/

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
	return { ACCELERATION_Mn_,ACCELERATION_Mt_ };
}

double Quadrotor::getT_go_(void) const {
	return T_go_;
}

double Quadrotor::getT_d_(void) const {
	return T_d_;
}

double Quadrotor::getGamma(void) const {
	return GAMMA;
}

double Quadrotor::getTHETA(void) const {
	return THETA;
}

double Quadrotor::getTHETA_d_(void) const {
	return THETA_d_;
}

double Quadrotor::getE_THETA_(void) const {
	return E_THETA_;
}

/*************************************************************/
//public set function
/************************************************************/

void Quadrotor::resetNu(void) {
	nu = 0;
}

void Quadrotor::resetK2(void) {
	k2 = k2_0;
}

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
	T_d_ = para;
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
	THETA_m_ = GAMMA - THETA;
}

void Quadrotor::updateE_theta(void) {
	E_THETA_ = THETA - THETA_d_;
}

void Quadrotor::updateTgo(void) {
	T_go_ = RANGE / (VELOCITY*cos(THETA_m_));
}

void Quadrotor::updateS(const double& t) {
	updateTgo();
	updateE_theta();
	S1 = mu * pow(abs(E_THETA_), rho)*sign(E_THETA_) - VELOCITY * sin(THETA_m_) / RANGE;
	S2 = t + T_go_ - T_d_;
}

void Quadrotor::updateAcc(vector<Quadrotor>& quad, const int& num, const double& t) {
	updateS(t);
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

void Quadrotor::updateState(void) {
	VEL = VEL + ACC * dt;
	VELOCITY = sqrt(VEL * VEL);
	updateGamma();
	POS = POS + VEL * dt;
	updateTheta();
	updateRange();
	updateTheta_m();
}

double Quadrotor::integral(double(*f)(const double&, const double&), const double& x1, const double& x2, double& prev, const double& step) {
	double value = f(x1, x2);
	double ans = (prev + value)*step / 2;
	return ans;
}

double Quadrotor::integral(double(*f)(const double&), const double& x, double& prev, const double& step) {
	double value = f(x);
	double ans = (prev + value)*step / 2;
	return ans;
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