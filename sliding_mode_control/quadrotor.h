#pragma once

#include<stdlib.h>
#include<vector>

#include"target.h"

class Quadrotor
{
private:
	int INDEX;
	coordinate POS;
	coordinate VEL;
	coordinate ACC;
	double VELOCITY;
	double ACCELERATION;
	double GAMMA;
	double THETA;
	double THETA_M;
	double Tgo;
	double Timp;
	double RANGE;
	double S;

	Target target;
	double ACC_EQ;
	double ACC_MCON;
	double ACC_SW;
	coordinate ACC_RUPLSION;

public:
	Quadrotor(void);
	~Quadrotor();
	
	//initial
	bool init(const std::vector<double> para);

	//public get function
	double getTimp(void) const;
	double getTgo(void) const;
	double getRange(void) const;
	coordinate getPos(void) const;
	coordinate getVel(void) const;
	double getVelocity(void) const;
	coordinate getAcc(void) const;
	double getAccleration(void) const;

	//public set function
	void setIndex(const int& x);
	void setVelocity(const double& para);
	void setAccleration(const double& para);
	void setTimp(const double& t);
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
	double H(const double& x);
	double sgmf(const double& x);
};