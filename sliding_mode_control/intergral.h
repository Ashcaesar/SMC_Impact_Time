#pragma once
#include"quadrotor.h"

//计算定积分
class intergral {
private:
	struct info {
		double value;
		double error;
	};
public:
	double DefiniteIntergral(double(*)(double), double, double, double);
};