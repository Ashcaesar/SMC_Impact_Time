#pragma once
#include"function.h"

extern const double PI;
extern const double k2_bar;

double sign(const double& x) {
	if (x == 0) return 0;
	return x > 0 ? 1.0 : -1.0;
}

double dnu(const double& alpha2, const double& S2) {
	return -alpha2 * pow(abs(S2), 1 / 3)*sign(S2);
}

double dk2(const double& S1) {
	return k2_bar * abs(S1);
}

void read_form(ifstream& file, vector<coordinate>& form, int num_of_quad) {
	if (!file) {
		cerr << "输入文件打开失败！" << endl;
		return;
	}
	string s;
	double tmp;
	stringstream sstream;
	for (int i = 0; i < num_of_quad; ++i) {
		getline(file, s);
		sstream << s;
		for (int j = 0; j < 2; ++j) {
			sstream >> tmp;
			if (j == 0) form[i].x = tmp;
			else form[i].y = tmp;
		}
		sstream.clear();
		sstream.str("");
	}
	file.close();
}

coordinate transition(const coordinate& prev, const coordinate& base, const double& theta) {
	double t = theta - PI / 2;
	coordinate ans;
	ans.x = base.x + prev.x*cos(t) - prev.y*sin(t);
	ans.y = base.y + prev.x*sin(t) + prev.y*cos(t);
	return ans;
}