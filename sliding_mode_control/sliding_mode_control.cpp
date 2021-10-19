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
	Target* leader = new Target;								//������Target����
	Target* assemble = new Target[num];							//����㣬Target����
	Target* virt = new Target[num];								//Ŀ��㣬Target����
	Quadrotor* quad = new Quadrotor[num];						//���˻���Quadrotor����
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
	for (int i = 0; i < num; i++) {
		coordinate trans = transition(form[i], leader->getPos(), leader->getGamma());
		assemble[i].init(trans);
		quad[i].setTarget(assemble[i]);

		vector<double> tmp;
		coordinate pos = assemble[i].getPos();
		tmp.push_back(pos.x);
		tmp.push_back(pos.y);
		tmp.push_back(leader->getVelocity());
		tmp.push_back(leader->getGamma() * 180 / PI);
		virt[i].init(tmp);
	}

	//��ȡ���˻��������
	read_data(quad_input, quad, num, 6);
	for (int i = 0; i < num; ++i) quad[i].setIndex(i);

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
	for (int i = 0; i < num; ++i) {
		if (quad[i].getTd() > tmax) tmax = quad[i].getTd();		//�����ʱ��
	}
	for (double time = 0; time <= tmax; time += dt) {
		for (int i = 0; i < num; ++i) {
			if (quad[i].getRange() > 1) {
				quad[i].updateAcc(quad, num, time);
				quad[i].updateState();
				assemble[i].updateState();
				quad[i].setTarget(assemble[i]);
			}
			write_data(quad_output, quad[i]);
			write_data(virtual_output, assemble[i]);
		}
		write_data(leader_output, leader[0]);
	}
	delete[] assemble;

	/*for (int i = 0; i < num; i++) {
		quad[i].setVelocity(0);
	}*/

	//��ӷ��в���
	/*
	int count = 0;
	double time = 0;
	for (double t = 0; t <= 200; t += dt) {
		leader->updateState();
		for (int i = 0; i < num; i++) {
			tar[i].setPos(transition(form[i], leader->getPos(), leader->getGamma()));
			tar[i].setVelocity(leader->getVelocity());
			tar[i].setGamma(leader->getGamma());
		}
		count++;
		if (count % 200==0) {
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
			quad[i].updateAcc(quad, num, t);
			quad[i].updateState();
			write_data(quad_output, quad[i]);
			write_data(target_output, tar[i]);
		}
		time += dt;
		write_data(leader_output, leader[0]);
	}
	*/

	quad_output.close();
	leader_output.close();
	virtual_output.close();
	delete[] quad;
	delete[] virt;

	return 0;
}