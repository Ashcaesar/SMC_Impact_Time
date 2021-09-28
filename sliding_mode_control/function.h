#pragma once
#include<stdlib.h>
#include<fstream>
#include<vector>

#include"coordinate.h"

using namespace std;

template<typename T> void read_data(ifstream& file, T* goal, int num_of_quad, int num_of_para);
template<typename T> void write_data(ofstream& file, T goal);

//计算旋转后坐标
coordinate transition(const coordinate prev, const coordinate base, const double& theta);

//文件读取
template<typename T> void read_data(ifstream& file, T* goal, int num_of_quad, int num_of_para) {
	if (!file) {
		cout << "文件打开失败！" << endl;
		return;
	}
	string s;
	double tmp;
	stringstream sstream;
	for (int i = 0; i < num_of_quad; i++) {
		getline(file, s);
		sstream << s;
		vector<double> para;
		for (int j = 0; j < num_of_para; j++) {
			sstream >> tmp;
			para.push_back(tmp);
		}
		goal[i].init(para);
		sstream.clear();
		sstream.str("");
	}
	file.close();
}

//文件写入
template<typename T> void write_data(ofstream& file, T goal) {
	coordinate pos = goal.getPos();
	coordinate vel = goal.getVel();
	file << pos.x << " " << pos.y << " " << vel.x << " " << vel.y << '\n';
}