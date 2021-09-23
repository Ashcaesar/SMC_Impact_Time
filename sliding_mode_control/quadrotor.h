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
	double Td;
	double RANGE;
	double S1;
	double S2;

	Target target;

public:
	Quadrotor(void);
	~Quadrotor();
	
	//initial
	bool init(const std::vector<double> para);

	//public get function
	double getRange(void) const;
	coordinate getPos(void) const;
	coordinate getVel(void) const;
	double getVelocity(void) const;
	coordinate getAcc(void) const;
	pair<double,double> getAccleration(void) const;

	//public set function
	void setIndex(const int& x);
	void setVelocity(const double& para);
	void setTarget(const Target& t);

	void updateRange(void);
	void updateGamma(void);
	void updateTheta(void);
	void updateTheta_m(void);
	void updateS(const double& t);
	void updateAcc(Quadrotor* quad, int num);
	void updateTgo(void);
	void updateState(void);
	
	double sign(const double& x);
	double phi(const double& x);
	double sgmf(const double& x);
};