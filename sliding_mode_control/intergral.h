#pragma once
#include"quadrotor.h"

//���㶨����
class intergral {
private:
	struct info {
		double value;
		double error;
	};
public:
	double DefiniteIntergral(double(*)(double), double, double, double);
};