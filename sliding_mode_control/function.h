#pragma once
#include<stdlib.h>
#include<fstream>
#include<vector>

#include"coordinate.h"

using namespace std;

template<typename T> void read_data(ifstream& file, T& goal, int num_of_quad, int num_of_para);
template<typename T> void write_data(ofstream& file, vector<T>& goal);

//������ת������
coordinate transition(const coordinate& prev, const coordinate& base, const double& theta);

//��ȡ����
void read_form(ifstream& file, vector<coordinate>& form, int num_of_quad) {
	if (!file) {
		cerr << "�����ļ���ʧ�ܣ�" << endl;
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

//�ļ���ȡ
template<typename T> void read_data(ifstream& file, T& goal, int num_of_quad, int num_of_para) {
	if (!file) {
		cerr << "�����ļ���ʧ�ܣ�" << endl;
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

//�ļ�д��
template<typename T> void write_data(ofstream& file, vector<T>& goal) {
	if (!file) {
		cerr << "����ļ���ʧ�ܣ�" << endl;
		return;
	}
	for (auto g : goal) {
		file << g.getPos().x << " " << g.getPos().y << " " << g.getVel().x << " " << g.getVel().y << '\n';
	}
}