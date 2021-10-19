#pragma once
#include<iostream>
#include<fstream>
#include<sstream>

#include"function.h"

extern const double PI;

coordinate transition(const coordinate& prev, const coordinate& base, const double& theta) {
	double t = theta - PI / 2;
	coordinate ans;
	ans.x = base.x + prev.x*cos(t) - prev.y*sin(t);
	ans.y = base.y + prev.x*sin(t) + prev.y*cos(t);
	return ans;
}