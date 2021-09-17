#include "coordinate.h"

coordinate operator +(const coordinate& a, const coordinate& b) {
	coordinate ans;
	ans.x = a.x + b.x;
	ans.y = a.y + b.y;
	return ans;
}

coordinate operator -(const coordinate& a, const coordinate& b) {
	coordinate ans;
	ans.x = a.x - b.x;
	ans.y = a.y - b.y;
	return ans;
}

coordinate operator *(const coordinate& a, const double& b) {
	coordinate ans;
	ans.x = a.x * b;
	ans.y = a.y * b;
	return ans;
}

double operator *(const coordinate& a, const coordinate& b) {
	double ans;
	ans = a.x * b.x + a.y * b.y;
	return ans;
}