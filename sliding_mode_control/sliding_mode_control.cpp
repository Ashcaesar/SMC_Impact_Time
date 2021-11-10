#include<stdlib.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<regex>

#include"target.h"
#include"uav.h"
#include"function.h"

extern const double g = 9.81;
extern const double PI = 3.1415926535;
extern const double dt = 0.01;
extern const double alpha1 = 0.45;
extern const double alpha2 = 0.2;
extern const double mu = 0.1;
extern const double k1 = 0.5;
extern const double k2_0 = 0.03;
extern const double k2_bar = 1.5;
extern const double lambda = 0.1;
extern const double tau = 0.01;
extern const double rho = 0.6;

extern const double K = 50;
extern const double M = 10;
extern const double N = 3;
extern const double P = 2;
extern const double Eps_1 = 0.001;
extern const double Eps_2 = 0.015;

using namespace std;

ostream& operator<<(ostream& os, const Uav& uav) {
	os.precision(3);
	os << "Uav " << uav.getIndex() << " data:\n"
		<< "Velocity =              [" << uav.getVel().x << "]\n"
		<< "                        [" << uav.getVel().y << "]\n"
		<< "Desired Impact Time =   [" << uav.getT_d_() << "]\n"
		<< "Final Impact Time =     [" << uav.getT_d_() - uav.getT_go_() << "]\n"
		<< "Desired Impact Angle =  [" << uav.getTHETA_d_() / PI * 180 << "]\n"
		<< "Final Impact Angle =    [" << uav.getGamma() / PI * 180 << "]\n"
		<< "**********************************"
		<< endl;
	os.precision();
	return os;
}

int main() {
	int num = 4;
	bool GatherMode = true;
	bool FormationMode = false;
	vector<Target> leader(1);			//长机,vector<Target>
	vector<Target> assemble(num);		//集结点,vector<Target>
	vector<Target> virts(num);			//目标点,vector<Target>
	vector<Uav> uavs(num);		//无人机,vector<Quadrotor>
	vector<coordinate> form(num);		//编队队形,vector<coordinate>
	
	ifstream uav_input_, form_input_, leader_input_;			//输入文件:无人机,队形,长机
	uav_input_.open("uav input.txt", ifstream::in);
	form_input_.open("form input.txt", ifstream::in);
	leader_input_.open("leader input.txt", ifstream::in);
	
	ofstream uav_output_, virtual_output_, leader_output_;		//输出文件:无人机,虚拟长机,长机
	uav_output_.open("uav output.txt", ofstream::out);
	virtual_output_.open("virtual output.txt", ofstream::out);
	leader_output_.open("leader output.txt", ofstream::out);
	
	read_data(leader_input_, leader, 1, 4);		//读取长机输入参数
	read_form(form_input_, form, 4);			//读取队形参数
	
	//初始化集结点和虚拟点参数
	for (int i = 0; i < num; ++i) {
		coordinate trans = transition(form.at(i), leader.at(0).getPos(), leader.at(0).getGamma());
		assemble.at(i).init(trans);
		uavs.at(i).setTarget(assemble.at(i));

		vector<double> tmp;
		coordinate pos = assemble.at(i).getPos();
		tmp.push_back(pos.x);
		tmp.push_back(pos.y);
		tmp.push_back(leader.at(0).getVelocity());
		tmp.push_back(leader.at(0).getGamma() * 180 / PI);
		virts.at(i).init(tmp);
	}

	read_data(uav_input_, uavs, num, 6);		//读取无人机输入参数
	for (int i = 0; i < num; ++i) uavs.at(i).setIndex(i);

	//为无人机分配最近的集结点
	vector<bool> isAllocated(num);
	for (auto& uav : uavs) {
		int temp = num;
		double minDis = DBL_MAX;
		for (int i = 0; i < num; ++i) {
			if (isAllocated[i]) continue;
			double dis = pow((uav.getPos() - assemble.at(i).getPos()).x, 2) + pow((uav.getPos() - assemble.at(i).getPos()).y, 2);
			temp = dis < minDis ? i : temp;
			minDis = min(minDis, dis);
		}
		uav.setIndex(temp);
		isAllocated[temp] = true;
	}

	//集结模式
	//同时约束时间和角度
	double tmax = 0.0;
	for (auto& uav : uavs) {
		tmax = max(tmax, uav.getT_d_());
	}
	for (double time = 0.0; time <= tmax; time += dt) {
		for (auto& quad : uavs) {
			if (quad.getRange() > 3.0) {
				quad.updateAcc(uavs, num, time, GatherMode);
				quad.updateState();
				assemble.at(quad.getIndex()).updateState();
				quad.setTarget(assemble.at(quad.getIndex()));
			}
		}
		write_data(uav_output_, uavs);
		write_data(virtual_output_, assemble);
		write_data(leader_output_, leader);
	}

	//编队模式
	/**/
	int count = 0;
	double time = 0;
	for (double t = 0; t <= 200; t += dt) {
		leader.at(0).updateState();
		for (int i = 0; i < num; ++i) {
			virts.at(i).setPos(transition(form.at(i), leader.at(0).getPos(), leader.at(0).getGamma()));
			virts.at(i).setVelocity(leader.at(0).getVelocity());
			virts.at(i).setGamma(leader.at(0).getGamma());
		}
		count++;
		if (count == 50) {
			time = 0;
			tmax = 0;
			for (auto& uav : uavs) {
				uav.setTarget(virts.at(uav.getIndex()));
				uav.updateRange();
				uav.updateGamma();
				uav.updateTheta();
				uav.updateTheta_m();
				uav.updateT_go_(FormationMode);
				tmax = max(tmax, uav.getT_go_());
			}
			for (auto& uav : uavs) {
				uav.setT_d_(tmax);
			}
		}

		if (abs(time - tmax) < 0.1) {
			time = 0;
			tmax = 0;
			for (auto& uav : uavs) {
				uav.setTarget(virts.at(uav.getIndex()));
				uav.updateRange();
				uav.updateGamma();
				uav.updateTheta();
				uav.updateTheta_m();
				uav.updateT_go_(FormationMode);
				tmax = max(tmax, uav.getT_go_());
			}
			for (auto& uav : uavs) {
				uav.setT_d_(tmax);
			}
		}

		for (auto& uav : uavs) {
			uav.updateAcc(uavs, num, time, FormationMode);
			uav.updateState();
			write_data(uav_output_, uavs);
			write_data(virtual_output_, virts);
		}
		time += dt;
		write_data(leader_output_, leader);
	}
	/**/

	//编队部分
	//仅约束时间
	/**
	double t = 0.0;
	for (double time = 0.0; time <= 200; time += dt) {
		leader.at(0).updateState();
		//更新虚拟长机参数
		for (int i = 0; i < num; ++i) {
			virts.at(i).setPos(transition(form.at(i), leader.at(0).getPos(), leader.at(0).getGamma()));
			virts.at(i).setVelocity(leader.at(0).getVelocity());
			virts.at(i).setGamma(leader.at(0).getGamma());
		}
		for (auto& uav : uavs) {
			uav.setTarget(virts.at(uav.getIndex()));
			uav.updateTgo();
			tmax = max(tmax, uav.getT_go_());
		}
		//跟随目标
		for (auto& uav : uavs) {
			uav.resetK2();
			uav.resetNu();
			uav.updateS(time);
			uav.updateAcc(uavs, num, t);
			uav.updateState();
		}
		write_data(uav_output_, uavs);
		write_data(virtual_output_, virts);
		write_data(leader_output_, leader);
		time += dt;
		t += dt;
	}
	/**/

	for (const Uav& uav : uavs) {
		cout << uav;
	}

	uav_output_.close();
	leader_output_.close();
	virtual_output_.close();

	return 0;
}