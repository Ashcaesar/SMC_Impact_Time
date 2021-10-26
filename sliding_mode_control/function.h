#pragma once
#include<stdlib.h>
#include<string>
#include<vector>
#include<iostream>
#include<fstream>
#include<sstream>
#include"coordinate.h"

using namespace std;

double sign(const double& x);
double dnu(const double& alpha2, const double& S2);
double dk2(const double& S1);
void read_form(ifstream& file, vector<coordinate>& form, int num_of_quad);
coordinate transition(const coordinate& prev, const coordinate& base, const double& theta);

template<typename T> void read_data(ifstream& file, T& goal, int num_of_quad, int num_of_para);
template<typename T> void write_data(ofstream& file, vector<T>& goal);

//文件读取
template<typename T> void read_data(ifstream& file, T& goal, int num_of_quad, int num_of_para) {
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
		vector<double> para;
		for (int j = 0; j < num_of_para; ++j) {
			sstream >> tmp;
			para.push_back(tmp);
		}
		goal.at(i).init(para);
		sstream.clear();
		sstream.str("");
	}
	file.close();
}

//文件写入
template<typename T> void write_data(ofstream& file, vector<T>& goal) {
	if (!file) {
		cerr << "输出文件打开失败！" << endl;
		return;
	}
	for (auto g : goal) {
		file << g.getPos().x << " " << g.getPos().y << " " << g.getVel().x << " " << g.getVel().y << '\n';
	}
}


