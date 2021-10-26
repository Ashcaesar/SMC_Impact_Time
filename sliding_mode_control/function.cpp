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