#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <chrono>
#include <thread>
#include "std_msgs/String.h"
#include <string>
#include <robot_control/MotorFreqs.h>


const double  pi = 3.1415926535897932384626433832795028841971;
using namespace std;

struct delta_wheel_speed {
	int right;
	int left;
};

struct robot_pos {
	double x;
	double y;
	double theta;
};

struct robot_state {
	double v;
	double omega;
};

struct lm {
	double x;
	double y;
};


class fuzzy {
public:
	fuzzy(vector<double> K ,vector<int> set_num, vector<int> set_type);
	vector<double> x;
	vector<double> K;
	vector<int> set_num;
	vector<int> set_type;
	struct set_value {
		int set;
		double value;
	};

	vector<vector<int>> rule1{ {
	{0,0,0,1,2,3,4},
	{0,0,1,1,2,3,4},
	{0,1,1,2,3,4,5},
	{0,1,2,3,4,5,6},
	{1,2,3,4,5,5,6},
	{2,3,4,5,5,6,6},
	{2,3,4,5,6,6,6}
	} };
	
	vector<vector<vector<int>>> rule2{ 
		{{0,1,2},{0,1,1},{0,0,0}},
		{{1,2,2},{1,1,1},{0,0,0}},
		{{2,2,2},{1,1,1},{0,0,0}}
	};

	vector<set_value> fuzzy_ante(double x, double k,int set_num,int type);
	set_value fuzzy_set(int i, double value,double k,int set_num,int first_pos);
	vector<fuzzy::set_value> fuzzy_min_case(vector<vector<set_value>> antes, size_t x_num);
	set_value fuzzy_min2(set_value a1, set_value a2);
	fuzzy::set_value fuzzy_min3(set_value ant1, set_value ant2, set_value ant3);
	double fuzzy_cons1(vector<set_value> outs);
	double fuzzy_calc(vector<double> x);

};

fuzzy::fuzzy(vector<double> K, vector<int> set_num, vector<int> set_type) {
	fuzzy::K = K;
	fuzzy::set_num = set_num;
	fuzzy::set_type = set_type;
}


class Map {
public:
	Map(vector<lm> lms);
	vector<lm> lms;
};

Map::Map(vector<lm> landmarks) {
	lms = landmarks;
}

class agent {
public:
	agent(robot_pos position, robot_state st, vector<lm> landmarks,int base_speed);
	robot_pos pos;
	robot_state state;
	int base;
	vector<lm> lms;
	int past_no = 0,now_no = 1, next_no = 2;
	double point_R = 0.1;
	vector<robot_pos> que{ {pos},{pos},{pos},{pos},{pos} };
	robot_control::MotorFreqs robot_control(fuzzy fuzzy1, fuzzy fuzzy2,double time);
	void oribit_point_judge();
	vector<double> input1();
	vector<double> input2(int ws1_delta);
	robot_pos output_pos(double v, double omega, double time);

};

agent::agent(robot_pos position, robot_state st, vector<lm> landmarks,int base_speed) {
	pos = { position.x,position.y,position.theta };
	state = { st.v,st.omega };
	lms = landmarks;
	base = base_speed;
}

fuzzy::set_value fuzzy::fuzzy_set(int i, double value, double k,int set_num,int first_pos) {
	double grade;
	double pos_value = (first_pos + i) * k;

	if (value <= pos_value - k) {
		grade = 0.0;
	}
	else if (value <= pos_value) {
		grade = (1 / k) * value + 1-first_pos -i;
	}
	else if (value <= pos_value + k) {
		grade = (- 1 / k) * value + first_pos + 1 + i;
	}
	else {
		grade = 0.0;

	}

	set_value result{ i,grade };
	return result;
}

vector<fuzzy::set_value> fuzzy::fuzzy_ante(double x, double k,int set_num,int type) {
	int i, most_left, most_right, first_pos;
	double base_pos;
	vector<set_value> value_sets;

	if (type == 0) {
		first_pos = -(set_num - 1) / 2;
		most_left = first_pos;
		most_right = -first_pos;
	}

	else if (type == 1) {
		first_pos = 0;
		most_left = first_pos;
		most_right = set_num-1;
	}

	else {
		first_pos = -set_num + 1;
		most_left = first_pos;
		most_right = 0;
	}

	for (i = 0; i < set_num; i++) {
		base_pos = abs((first_pos + i) * k - x);
		if ( base_pos < k) {
			if(base_pos == 0){ value_sets.emplace_back(set_value{ i,1.0 });}
			else {
				value_sets.emplace_back(fuzzy_set(i, x, k, set_num, first_pos));
				value_sets.emplace_back(fuzzy_set(i + 1, x, k, set_num, first_pos));
			}
			break;
		}
		else if (base_pos == k) {
			value_sets.emplace_back(set_value{ i+1,1.0 });
			break;
		}
		else if (x <= most_left * k) {
			value_sets.emplace_back(set_value{ 0,1.0 });
			break;
		}
		else if (x >= most_right * k) {
			value_sets.emplace_back(set_value{ set_num-1,1.0 });
			break;
		}
	}

	return  value_sets;
}

vector<fuzzy::set_value> fuzzy::fuzzy_min_case(vector<vector<set_value>> antes ,size_t x_num) {
	int i, j, k;
	vector<set_value> outs;

	if (x_num == 2) {
		for (j = 0; j < antes[0].size(); j++) {
			for (i = 0; i < antes[1].size(); i++) {	
				outs.emplace_back(fuzzy_min2(antes[0][j], antes[1][i]));
			}
		}
	}

	else if (x_num == 3) {
		for (k = 0; k < antes[0].size();k++) {
			for (j = 0; j < antes[1].size(); j++) {
				for (i = 0; i < antes[2].size(); i++) { 
					outs.emplace_back(fuzzy_min3(antes[0][k], antes[1][j], antes[2][i]));
				}
			}
		}
	}

	return outs;
}

fuzzy::set_value fuzzy::fuzzy_min2(set_value ant1, set_value ant2) {
	set_value out_set;
	out_set.set = rule1[ant2.set][ant1.set];

	if (ant1.value >= ant2.value) {out_set.value = ant2.value;}
	else if (ant1.value < ant2.value) {out_set.value = ant1.value;}
	return out_set;
}

fuzzy::set_value fuzzy::fuzzy_min3(set_value ant3, set_value ant2,set_value ant1) {
	set_value out_set;
	out_set.set = rule2[ant3.set][ant2.set][ant1.set];
	if (ant1.value <= ant2.value && ant1.value <= ant3.value) {out_set.value = ant1.value;}
	else if (ant2.value <= ant1.value && ant2.value <= ant3.value) {out_set.value = ant2.value;}
	else if (ant3.value <= ant1.value && ant3.value <= ant2.value) {out_set.value = ant3.value;}
	return out_set;
}

double fuzzy::fuzzy_cons1(vector<set_value> outs) {
	int i,first_pos;
	vector<double> out(set_num.back(), 0);

	if (set_type.back() == 0) {first_pos = -(set_num.back() - 1) / 2; }
	else if (set_type.back() == 1) { first_pos = 0; }
	else {first_pos = -set_num.back() + 1; }

	for (i = 0; i < outs.size(); i++) {
		if (out[outs[i].set] != 0) {
			if (outs[i].value > out[outs[i].set]) {
				out[outs[i].set] = outs[i].value;
			}
		}
		else { out[outs[i].set] = outs[i].value; }
	}
	double deno = 0, nume = 0, output;
	for (i = 0; i < set_num.back(); i++) {
		deno += out[i];
		nume += out[i] * (first_pos + i) * K.back();
	}

	output = nume / deno;
	return output;
}



double fuzzy::fuzzy_calc(vector<double> x) {
	int i;
	vector<vector<set_value>> antes;
	vector<set_value> outs;

	for (i = 0; i < x.size(); i++) {
		antes.emplace_back(fuzzy_ante(x[i], K[i], set_num[i],set_type[i]));
	}
	outs = fuzzy_min_case(antes,x.size());
	double output_speed = fuzzy_cons1(outs);
	return output_speed;
}

void agent::oribit_point_judge() {
	int i;

	for (i = 0; i < que.size(); i++) {
		if (sqrt(pow((que[i].x - lms[now_no].x), 2) + pow((que[i].y - lms[now_no].y), 2)) <= point_R) {
			past_no += 1;
			now_no += 1;
			next_no += 1;
			if (past_no == lms.size()) {
				past_no = 0;
			}
			if (now_no == lms.size()) {
				now_no = 0;
			}
			if (next_no == lms.size()) {
				next_no = 0;
			}
			break;
		}
	}

}

vector<double> agent::input1() {
	double past_mark_x, past_mark_y, now_mark_x, now_mark_y;
	double dx, dy, alpha;
	double Sx=0, thetax;

	past_mark_x = lms[past_no].x;
	past_mark_y = lms[past_no].y;
	now_mark_x = lms[now_no].x;
	now_mark_y = lms[now_no].y;

	dx = now_mark_x - past_mark_x;
	dy = now_mark_y - past_mark_y;
	alpha = atan2(dy, dx);
	thetax = alpha - pos.theta;

	if (thetax < -pi) {
		thetax += 2 * pi;
	}

	if (past_mark_x == now_mark_x) {
		if (now_mark_y > past_mark_y) {
			Sx = (pos.x - past_mark_x) * 1000;
		}
		else if (past_mark_y > now_mark_y) {
			Sx = -(pos.x - past_mark_x) * 1000;
		}
	}
	else if (past_mark_y == now_mark_y) {
		if (now_mark_x > past_mark_x) {
			Sx = -(pos.y - past_mark_y) * 1000;
		}
		else if (past_mark_x > now_mark_x) {
			Sx = (pos.y - past_mark_y) * 1000;
		}
	}
	else {
		double a = now_mark_y - past_mark_y;
		double  b = past_mark_x - now_mark_x;
		double c = now_mark_x * past_mark_y - past_mark_x * now_mark_y;
		Sx = (a * pos.x + b * pos.y + c) *1000 / sqrt(a * a + b * b);
	
	}
	vector<double> x1({ thetax,Sx });

	return x1;
}

vector<double> agent::input2(int omg) {
	double S, theta2, p1, p2, p3;
	S = sqrt(pow((lms[now_no].x - pos.x), 2) + pow((lms[now_no].y - pos.y), 2)) * 1000;
	p1 = sqrt(pow((lms[now_no].x - pos.x), 2) + pow((lms[now_no].y - pos.y), 2));
	p2 = sqrt(pow((lms[next_no].x - pos.x), 2) + pow((lms[next_no].y - pos.y), 2));
	p3 = sqrt(pow((lms[now_no].x - lms[next_no].x), 2) + pow((lms[now_no].y - lms[next_no].y), 2));
	theta2 = acos((p1 * p1 + p3 * p3 - p2 * p2) / (2 * p1 * p3 + 10e-10)); 
	vector<double> x2 ({ theta2,double(omg),S });

	return x2;
}

robot_pos agent::output_pos(double v,double omega,double time) {

	double t0 = pos.theta;
	robot_pos next_pos;
	if (abs(omega) < 1e-10) {
		next_pos = {
			pos.x + v * cos(t0) * time,
			pos.y + v * sin(t0) * time,
			pos.theta + omega * time
		};
	}
	else {
		next_pos = {
			pos.x + v  * (sin(t0 + omega * time) - sin(t0)) / omega,
			pos.y + v  * (-cos(t0 + omega * time) + cos(t0)) / omega,
			pos.theta + omega * time
		};
	}

	if (next_pos.theta >= 2 * pi) {
		next_pos.theta = next_pos.theta - 2 * pi;
	}
	else if (next_pos.theta < 0) {
		next_pos.theta = next_pos.theta + 2 * pi;
	}

	return next_pos;
}



robot_control::MotorFreqs agent::robot_control(fuzzy fuzzy1,fuzzy fuzzy2 ,double time) {
	int ws1_delta;
	int ws2_delta;
	delta_wheel_speed ws;
	robot_control::MotorFreqs value;
	robot_pos next_pos;
	oribit_point_judge();
	vector<double> x1 = input1();
	ws1_delta = int(fuzzy1.fuzzy_calc(x1));

	vector<double> x2 = input2(ws1_delta);
	ws2_delta = int(fuzzy2.fuzzy_calc(x2));

	value.left_hz = base - ws1_delta + ws2_delta;
	value.right_hz = base + ws1_delta + ws2_delta;

	if(value.left_hz>1500){
		value.left_hz = 1500;
	}
	else if(value.left_hz < -1500){
		value.left_hz = -1500;
	}

	if(value.right_hz>1500){
		value.right_hz = 1500;
	}
	else if(value.right_hz<-1500){
		value.right_hz = -1500;
	}


	double v = (value.left_hz+value.right_hz)*9*pi/(80000*2);
	double omega = (-value.left_hz + value.right_hz)*pi/(400*2);

	next_pos = output_pos(v,omega,time);

	que.erase(que.begin(), que.begin() + 1);
	que.emplace_back(pos);
	pos = { next_pos };
	return value;
}



int main(int argc,char** argv){

	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	vector<double> K1({ 0.393,189,108 });
	vector<int> set_num1({7,7,7});
	vector<int> set_type1({ 0,0,0 });

	vector<double> K2({ 1.23,138,34.8,232 });
	vector<int> set_num2({ 3,3,3,3 });
	vector<int> set_type2({ 1,1,1,-1 });

	fuzzy fuzzy1(K1,set_num1,set_type1);
	fuzzy fuzzy2(K2, set_num2, set_type2);

	vector<lm> landmarks{ {0,0},{0,4.4},{-2.3,4.4},{-2.3,0} };
	Map m(landmarks);

	int base_speed = 600;
	robot_pos robo_pos = { 0,0,pi/2};
	robot_state init_state = { base_speed*9*pi/80000,0 };

	agent agent1(robo_pos, init_state, landmarks,base_speed);

	double time_sum = 0;
	double time_delta = 0.5;
	double time_hz = 1/time_delta;
	ros::init(argc,argv,"control_node");
	ros::NodeHandle n;
	ros::Publisher control_pub = n.advertise<robot_control::MotorFreqs>("motor_raw",10);

	
	ros::Rate loop_rate(time_hz);
	ros::Time t_start = ros::Time::now();
	while (ros::ok()) {
		//ROS_INFO("%lf",time_sum);
		robot_control::MotorFreqs message = agent1.robot_control(fuzzy1, fuzzy2, time_delta);
		control_pub.publish(message);
		ROS_INFO("now_no = %d	left = %d	right = %d",agent1.now_no,message.left_hz,message.right_hz);
		loop_rate.sleep();
		time_sum = (ros::Time::now()-t_start).toSec();

	}

	return 0;
}


