#pragma once

#include<stdlib.h>
#include<vector>
#include<utility>

#include"target.h"

using namespace std;

class Quadrotor
{
private:
	int INDEX;
	coordinate POS;
	coordinate VEL;
	coordinate ACC;
	double VELOCITY;
	double ACCELERATION;
	double ACCELERATION_Mn;
	double ACCELERATION_Mt;
	double GAMMA;
	double THETA;
	double THETA_M;
	double THETA_d;
	double e_THETA;
	double Tgo;
	double T_d;
	double RANGE;
	double S1;
	double S2;
	Target target;
	double nu;
	double dnu(void);
	double dnu(const double&);
	double dk2(double);

public:	
	Quadrotor(void);
	~Quadrotor();
	//initial
	bool init(const std::vector<double>& para);

	//public get function
	double getRange(void) const;
	coordinate getPos(void) const;
	coordinate getVel(void) const;
	double getVelocity(void) const;
	coordinate getAcc(void) const;
	pair<double,double> getAccleration(void) const;
	double getTgo(void) const;
	double getTd(void) const;

	//public set function
	void setIndex(const int&);
	void setVelocity(const double&);
	void setTarget(const Target&);
	void setTd(const double&);

	void updateRange(void);
	void updateGamma(void);
	void updateTheta(void);
	void updateTheta_m(void);
	void updateE_theta(void);
	void updateS(const double&);
	void updateAcc(Quadrotor*, const int&, const double&);
	void updateTgo(void);
	void updateState(void);
	double integral_Nu(const double&, const double&, const double&);
	double integral_k2(const double&, const double&, const double&);
	
	double sign(const double&);
	double phi(const double&);
	double sgmf(const double&);
};