#pragma once

#include<stdlib.h>
#include<vector>

#include"coordinate.h"

class Target
{
private:
	coordinate POS;
	coordinate POS_TP;
	coordinate VEL;

	double VELOCITY;
	double GAMMA;
public:
	Target(void);
	virtual ~Target();

	//initial
	virtual bool init(std::vector<double>&);
	virtual bool init(coordinate&);

	//public get function
	coordinate getPos(void) const;
	coordinate getPos_TP(void) const;
	coordinate getVel(void) const;
	double getVelocity(void) const;
	double getGamma(void) const;

	//public set function
	void setGamma(const double&);
	void setPos(const coordinate&);
	void setPos_TP(const double&);
	void setVel(const coordinate&);
	void setVelocity(const double&);
	void updateState(void);
};