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
	double ACCELERATION_Mn_;
	double ACCELERATION_Mt_;
	double GAMMA;
	double THETA;
	double THETA_m_;
	double THETA_d_;
	double E_THETA_;
	double T_go_;
	double T_d_;
	double RANGE;
	double S1;
	double S2;
	Target target;
	double nu;
	double dnu(void);
	double dnu(const double&);
	double temp_dnu_;
	double dk2(double);

public:	
	Quadrotor(void);
	~Quadrotor();
	//initial
	bool init(const std::vector<double>& para);

	//public get function
	int getIndex(void) const;
	double getRange(void) const;
	coordinate getPos(void) const;
	coordinate getVel(void) const;
	double getVelocity(void) const;
	coordinate getAcc(void) const;
	pair<double,double> getAccleration(void) const;
	double getT_go_(void) const;
	double getT_d_(void) const;
	double getGamma(void) const;
	double getTHETA(void) const;
	double getTHETA_d_(void) const;
	double getE_THETA_(void) const;

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
	void updateAcc(vector<Quadrotor>&, const int&, const double&);
	void updateTgo(void);
	void updateState(void);
	double integral_Nu(const double&, const double&, const double&);
	double integral_k2(const double&, const double&, const double&);
	
	double sign(const double&);
	double phi(const double&);
	double sgmf(const double&);	
};