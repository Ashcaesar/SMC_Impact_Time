#pragma once
struct coordinate {
	double x, y;
	friend coordinate operator +(const coordinate& a, const coordinate& b);
	friend coordinate operator -(const coordinate& a, const coordinate& b);
	friend coordinate operator *(const coordinate& a, const double& b);
	friend double operator *(const coordinate& a, const coordinate& b);
};

