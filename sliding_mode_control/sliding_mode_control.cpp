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

ostream& operator<<(ostream& os, const Quadrotor& quad) {
	os.precision(3);
	os << "Quadrotor " << quad.getIndex() << " data:\n"
		<< "Velocity =              [" << quad.getVel().x << "]\n"
		<< "                        [" << quad.getVel().y << "]\n"
		<< "Desired Impact Time =   [" << quad.getT_d_() << "]\n"
		<< "Final Impact Time =     [" << quad.getT_d_() - quad.getT_go_() << "]\n"
		<< "Desired Impact Angle =  [" << quad.getTHETA_d_() / PI * 180 << "]\n"
		<< "Final Impact Angle =    [" << quad.getGamma() / PI * 180 << "]\n"
		<< "**********************************"
		<< endl;
	os.precision();
	return os;
}

int main() {
	int num = 4;
	vector<Target> leader(1);			//长机,vector<Target>
	vector<Target> assemble(num);		//集结点,vector<Target>
	vector<Target> virt(num);			//目标点,vector<Target>
	vector<Quadrotor> quad(num);		//无人机,vector<Quadrotor>
	vector<coordinate> form(num);		//编队队形,vector<coordinate>
	
	ifstream quad_input_, form_input_, leader_input_;			//输入文件:无人机,队形,长机
	leader_input_.open("leader input.txt", ifstream::in);
	form_input_.open("form input.txt", ifstream::in);
	quad_input_.open("quad input.txt", ifstream::in);

	read_data(leader_input_, leader, 1, 4);		//读取长机输入参数
	read_form(form_input_, form, 4);			//读取队形参数

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

	read_data(quad_input_, quad, num, 6);		//读取无人机输入参数
	for (int i = 0; i < num; ++i) quad.at(i).setIndex(i);

	ofstream quad_output_, virtual_output_, leader_output_;		//输出文件:无人机,虚拟长机,长机
	leader_output_.open("leader output.txt", ofstream::out);
	quad_output_.open("quad output.txt", ofstream::out);
	virtual_output_.open("virtual output.txt", ofstream::out);

	//集结部分
	double tmax = 0;
	for (auto q : quad) {
		tmax = max(tmax, q.getT_d_());
	}
	for (double time = 0; time <= tmax; time += dt) {

		for (int i = 0; i < num; ++i) {
			if (quad.at(i).getRange() > 3) {
				quad.at(i).updateAcc(quad, num, time);
				quad.at(i).updateState();
				assemble.at(i).updateState();
				quad.at(i).setTarget(assemble.at(i));
			}

		}
		write_data(quad_output_, quad);
		write_data(virtual_output_, assemble);
		write_data(leader_output_, leader);
	}

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
		for (auto q : quad) {
			q.setTarget(virt.at(q.getIndex()));
			q.updateTgo();
			tmax = max(tmax, q.getT_go_());
		}
		//跟随目标
		for (auto q : quad) {
			q.updateS(time);
			q.updateAcc(quad, num, t);
			q.updateState();
		}
		write_data(quad_output_, quad);
		write_data(virtual_output_, virt);
		write_data(leader_output_, leader);
		time += dt;
		t += dt;
	}
	/**/

	for (int i = 0; i < num; ++i) {
		cout << quad.at(i);
	}

	quad_output_.close();
	leader_output_.close();
	virtual_output_.close();

	return 0;
}