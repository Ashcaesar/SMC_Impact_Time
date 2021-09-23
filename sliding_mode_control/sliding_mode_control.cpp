#include<stdlib.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<regex>

#include"target.h"
#include"quadrotor.h"
#include"function.h"

extern const double g = 9.81;
extern const double PI = 3.1415926535;
extern const double dt = 0.01;
extern const double alpha1 = 0.45;
extern const double alpha2 = 0.2;
extern const double mu = 0.1;
extern const double k1 = 0.5;
extern const double lambda = 0.1;
extern const double tau = 0.01;
extern const double rho = 0.6;

using namespace std;

int main() {
	//int num;
	//cout << "输入无人机数量:" << endl;
	//cin >> num;
	int num = 4;

	ifstream quad_input, form_input, leader_input;				//输入文件
	ofstream quad_output, target_output, leader_output;			//输出文件
	Target* leader = new Target;								//长机，Target对象
	Target* assemble = new Target[num];							//集结点，Target对象
	Target* tar = new Target[num];								//目标点，Target对象
	Quadrotor* quad = new Quadrotor[num];						//无人机，Quadrotor对象
	vector<coordinate> form(num);								//编队队形
	leader_input.open("leader input.txt", ifstream::in);
	form_input.open("form input.txt", ifstream::in);
	quad_input.open("quad input.txt", ifstream::in);

	//读取长机输入参数
	read_data(leader_input, leader, 1, 4);
	
	//读取队形参数
	string s;
	stringstream sstream;
	double tmp;
	for (int i = 0; i < num; i++) {
		getline(form_input, s);
		sstream << s;
		for (int j = 0; j < 2; j++) {
			sstream >> tmp;
			if (j == 0) form[i].x = tmp;
			else form[i].y = tmp;
		}
		sstream.clear();
		sstream.str("");
	}
	form_input.close();

	//初始化集结点和目标点参数
	for (int i = 0; i < num; i++) {
		vector<double> data;
		coordinate trans = transition(form[i], leader->getPos(), leader->getGamma());
		data.push_back(trans.x);
		data.push_back(trans.y);
		data.push_back(0);
		data.push_back(0);
		assemble[i].init(data);
		quad[i].setTarget(assemble[i]);

		vector<double> tmp;
		coordinate pos = assemble[i].getPos();
		tmp.push_back(pos.x);
		tmp.push_back(pos.y);
		tmp.push_back(leader->getVelocity());
		tmp.push_back(leader->getGamma() * 180 / PI);
		tar[i].init(tmp);
	}

	//读取无人机输入参数
	read_data(quad_input, quad, num, 6);
	for (int i = 0; i < num; i++) quad[i].setIndex(i);

	//打开输出文件
	leader_output.open("leader output.txt", ofstream::out);
	quad_output.open("quad output.txt", ofstream::out);
	target_output.open("target output.txt", ofstream::out);
	if (!quad_output) {
		cout << "quadrotor输出文件打开失败！" << endl;
		return 0;
	}
	if (!target_output) {
		cout << "target输出文件打开失败！" << endl;
		return 0;
	}
	if (!leader_output) {
		cout << "leader输出文件打开失败！" << endl;
		return 0;
	}
	
	//基于比例导引和期望时间的滑模面控制
	//集结部分
	double tmax = 0;
	for (int i = 0; i < num; i++) {
		if (quad[i].getTd() > tmax) tmax = quad[i].getTd();
	}
	for (double t = 0; t < tmax; t += dt) {
		for (int i = 0; i < num; i++) {
			if (quad[i].getRange() > 1) {
				quad[i].updateTgo();
				quad[i].updateS(t);
				assemble[i].updateTP(quad[i].getTgo());
				quad[i].updateAcc(quad, num);
				quad[i].updateState();
				assemble[i].updateState();
				quad[i].setTarget(assemble[i]);
			}
			write_data(quad_output, quad[i]);
			write_data(target_output, assemble[i]);
		}
		write_data(leader_output, leader[0]);
	}
	delete[] assemble;

	/*for (int i = 0; i < num; i++) {
		quad[i].setVelocity(0);
	}*/

	//编队飞行部分
	int count = 0;
	double time = 0;
	for (double t = 0; t <= 200; t += dt) {
		leader->updateState();
		for (int i = 0; i < num; i++) {
			tar[i].setPos(transition(form[i], leader->getPos(), leader->getGamma()));
			tar[i].setVelocity(leader->getVelocity());
			tar[i].setGamma(leader->getGamma());
			tar[i].updateTP(quad[i].getTgo());
		}
		count++;
		if (count == 50) {
			time = 0;
			tmax = 0;
			for (int i = 0; i < num; i++) {
				quad[i].setTarget(tar[i]);
				quad[i].updateRange();
				quad[i].updateGamma();
				quad[i].updateTheta();
				quad[i].updateTheta_m();
				quad[i].updateTgo();
				if (quad[i].getTgo() > tmax) tmax = quad[i].getTgo();
				for (int i = 0; i < num; i++) {
					quad[i].setTd(tmax);
				}
			}
		}

		if (abs(time - tmax) < 0.1) {
			time = 0;
			tmax = 0;
			for (int i = 0; i < num; i++) {
				quad[i].setTarget(tar[i]);
				quad[i].updateRange();
				quad[i].updateGamma();
				quad[i].updateTheta();
				quad[i].updateTheta_m();
				quad[i].updateTgo();
				if (quad[i].getTgo() > tmax) tmax = quad[i].getTgo();
				for (int i = 0; i < num; i++) {
					quad[i].setTd(tmax);
				}
			}
		}
		for (int i = 0; i < num; i++) {
			quad[i].updateS(time);
			quad[i].updateAcc(quad, num);
			quad[i].updateState();
			write_data(quad_output, quad[i]);
			write_data(target_output, tar[i]);
		}
		time += dt;
		write_data(leader_output, leader[0]);
	}

	quad_output.close();
	leader_output.close();
	target_output.close();
	delete[] quad;
	delete[] tar;

	return 0;
}