#pragma once
#include"intergral.h"

double intergral::DefiniteIntergral(double(*f)(double), double lower, double upper, double step) {
	double ans = 0;
	double value1 = (*f)(lower), value2, area;
	for (double i = lower; i < upper; i += step) {
		value2 = (*f)(i + step);
		area = (value1 + value2)*step / 2;
		value1 = value2;
		ans += area;
	}
	return ans;
}