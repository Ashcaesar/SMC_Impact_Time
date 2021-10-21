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
	//cout << "�������˻�����:" << endl;
	//cin >> num;
	int num = 4;

	ifstream quad_input, form_input, leader_input;				//�����ļ�:���˻�,����,����
	ofstream quad_output, virtual_output, leader_output;		//����ļ�:���˻�,���ⳤ��,����
	vector<Target> leader(1);
	/*Target* leader = new Target;*/								//������Target����
	vector<Target> assemble(num);
	/*Target* assemble = new Target[num];*/							//����㣬Target����
	vector<Target> virt(num);
	/*Target* virt = new Target[num];*/								//Ŀ��㣬Target����
	vector<Quadrotor> quad(num);
	/*Quadrotor* quad = new Quadrotor[num];*/						//���˻���Quadrotor����
	vector<coordinate> form(num);								//��Ӷ���
	leader_input.open("leader input.txt", ifstream::in);
	form_input.open("form input.txt", ifstream::in);
	quad_input.open("quad input.txt", ifstream::in);

	//��ȡ�����������
	read_data(leader_input, leader, 1, 4);

	//��ȡ���β���
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

	//��ʼ����������������
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

	//��ȡ���˻��������
	read_data(quad_input, quad, num, 6);
	for (int i = 0; i < num; ++i) quad.at(i).setIndex(i);

	//������ļ�
	leader_output.open("leader output.txt", ofstream::out);
	if (!leader_output) {
		cerr << "leader����ļ���ʧ�ܣ�" << endl;
		return 0;
	}

	quad_output.open("quad output.txt", ofstream::out);
	if (!quad_output) {
		cerr << "quadrotor����ļ���ʧ�ܣ�" << endl;
		return 0;
	}

	virtual_output.open("virtual output.txt", ofstream::out);
	if (!virtual_output) {
		cerr << "virtual����ļ���ʧ�ܣ�" << endl;
		return 0;
	}

	//���Ჿ��
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

	//��ӷ��в���
	/**/
	double t = 0;
	for (double time = 0; time <= 200; time += dt) {
		leader.at(0).updateState();
		//�������ⳤ������
		for (int i = 0; i < num; ++i) {
			virt.at(i).setPos(transition(form.at(i), leader.at(0).getPos(), leader.at(0).getGamma()));
			virt.at(i).setVelocity(leader.at(0).getVelocity());
			virt.at(i).setGamma(leader.at(0).getGamma());
		}
		//����Ŀ���
		if (time == 10 || (t - tmax) < 0.1) {
			t = 0;
			tmax = 0;
			for (auto q : quad) {
				q.setTarget(virt.at(q.getIndex()));
				q.updateTgo();
				tmax = max(tmax, q.getTgo());
			}
		}
		//����Ŀ��
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