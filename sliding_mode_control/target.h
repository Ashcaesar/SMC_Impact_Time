#pragma once

#include<stdlib.h>
#include<vector>

#include"coordinate.h"

class Target
{
private:
	coordinate POS;
	coordinate VEL;
	coordinate TP;

	double VELOCITY;
	double GAMMA;
public:
	Target(void);
	virtual ~Target();

	//initial
	virtual bool init(std::vector<double> para);

	//public get function
	coordinate getPos(void) const;
	coordinate getVel(void) const;
	double getVelocity(void) const;
	coordinate getTP(void) const;
	double getGamma(void) const;

	//public set function
	void setGamma(const double& para);
	void setPos(const coordinate& para);
	void setVel(const coordinate& para);
	void setVelocity(const double& para);
	void updateState(void);
	void updateTP(const double& tgo);
};