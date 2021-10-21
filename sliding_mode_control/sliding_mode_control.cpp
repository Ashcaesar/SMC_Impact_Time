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
extern const double k2_bar = 1.5;
extern const double lambda = 0.1;
extern const double tau = 0.01;
extern const double rho = 0.6;

using namespace std;

int main() {
	//int num;
	//cout << "输入无人机数量:" << endl;
	//cin >> num;
	int num = 4;

	ifstream quad_input, form_input, leader_input;				//输入文件:无人机,队形,长机
	ofstream quad_output, virtual_output, leader_output;		//输出文件:无人机,虚拟长机,长机
	vector<Target> leader(1);
	/*Target* leader = new Target;*/								//长机，Target对象
	vector<Target> assemble(num);
	/*Target* assemble = new Target[num];*/							//集结点，Target数组
	vector<Target> virt(num);
	/*Target* virt = new Target[num];*/								//目标点，Target数组
	vector<Quadrotor> quad(num);
	/*Quadrotor* quad = new Quadrotor[num];*/						//无人机，Quadrotor数组
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
	for (int i = 0; i < num; ++i) {
		getline(form_input, s);
		sstream << s;
		for (int j = 0; j < 2; ++j) {
			sstream >> tmp;
			if (j == 0) form[i].x = tmp;
			else form[i].y = tmp;
		}
		sstream.clear();
		sstream.str("");
	}
	form_input.close();

	//初始化集结点和虚拟点参数
	for (int i = 0; i < num; ++i) {
		coordinate trans = transition(form.at(i), leader.at(0).getPos(), leader.at(0).getGamma());
		assemble.at(i).init(trans);
		quad.at(i).setTarget(assemble.at(i));

		vector<double> tmp;
		coordinate pos = assemble.at(i).getPos();
		tmp.push_back(pos.x);
		tmp.push_back(pos.y);
		tmp.push_back(leader.at(0).getVelocity());
		tmp.push_back(leader.at(0).getGamma() * 180 / PI);
		virt.at(i).init(tmp);
	}

	//读取无人机输入参数
	read_data(quad_input, quad, num, 6);
	for (int i = 0; i < num; ++i) quad.at(i).setIndex(i);

	//打开输出文件
	leader_output.open("leader output.txt", ofstream::out);
	if (!leader_output) {
		cerr << "leader输出文件打开失败！" << endl;
		return 0;
	}

	quad_output.open("quad output.txt", ofstream::out);
	if (!quad_output) {
		cerr << "quadrotor输出文件打开失败！" << endl;
		return 0;
	}

	virtual_output.open("virtual output.txt", ofstream::out);
	if (!virtual_output) {
		cerr << "virtual输出文件打开失败！" << endl;
		return 0;
	}

	//集结部分
	double tmax = 0;
	for (auto q : quad) {
		tmax = max(tmax, q.getTd());
	}
	for (double time = 0; time <= tmax; time += dt) {
		for (int i = 0; i < num; ++i) {
			if (quad.at(i).getRange() > 1) {
				quad.at(i).updateAcc(quad, num, time);
				quad.at(i).updateState();
				assemble.at(i).updateState();
				quad.at(i).setTarget(assemble.at(i));
			}
			
		}
		write_data(quad_output, quad);
		write_data(virtual_output, assemble);
		write_data(leader_output, leader);
	}

	/**/for (auto& q : quad) { q.setVelocity(0); }/**/

	//编队飞行部分
	/**/
	double t = 0;
	for (double time = 0; time <= 200; time += dt) {
		leader.at(0).updateState();
		//更新虚拟长机参数
		for (int i = 0; i < num; ++i) {
			virt.at(i).setPos(transition(form.at(i), leader.at(0).getPos(), leader.at(0).getGamma()));
			virt.at(i).setVelocity(leader.at(0).getVelocity());
			virt.at(i).setGamma(leader.at(0).getGamma());
		}
		//重置目标点
		if (time == 10 || (t - tmax) < 0.1) {
			t = 0;
			tmax = 0;
			for (auto q : quad) {
				q.setTarget(virt.at(q.getIndex()));
				q.updateTgo();
				tmax = max(tmax, q.getTgo());
			}
		}
		//跟随目标
		for (auto q : quad) {
			q.updateS(time);
			q.updateAcc(quad, num, t);
			q.updateState();
		}
		write_data(quad_output, quad);
		write_data(virtual_output, virt);
		write_data(leader_output, leader);
		time += dt;
		t += dt;
	}
	/**/

	quad_output.close();
	leader_output.close();
	virtual_output.close();

	return 0;
}