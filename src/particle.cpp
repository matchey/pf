
#include<ros/ros.h>
#include"pf/particle.h"

using namespace std;

Particle::Particle()
	:x(0.0), y(0.0), yaw(0.0),
	weight(1.0), flag_fast(false), 
	init_x(0.0), init_y(0.0), edge_num(0),
	flag(false), x_new(0.0), y_new(0.0), dist_r_last(0.0), cnt_yaw(0.0), edge_last(-1)
{
}

Particle::Particle(double _x, double _y, double _yaw, double _mu, double noise)
		: x(_x), y(_y), yaw(_yaw){}

void Particle::setXY(double _x, double _y)
{
	x = _x; y = _y;
}

void Particle::setWeight(double w){weight = w;}
void Particle::setInit(double _x, double _y)
{
	init_x=_x; init_y=_y;
}

void Particle::setEdge(int edge){edge_num = edge;}

void Particle::setFast(bool fast){flag_fast = fast;}

double Particle::getX(){return x;}
double Particle::getY(){return y;}
double Particle::getYaw(){return yaw;}

double Particle::getWeight(){return weight;}

double Particle::getEdge(){return edge_num;}

double Particle::getDist(){
	distance = pow(x-init_x, 2) + pow(y-init_y, 2);
	return distance;
}

double Particle::distSquared(double _x, double _y)
{
	return pow(_x-init_x, 2)+pow(_y-init_y,2);
}

void Particle::sense()
{
	// landmark??
}

void Particle::move(double dist, double direction)
{
	// if(flag_fast) dist *= 5.1;
	yaw = direction;
	x += dist * cos(yaw);
	y += dist * sin(yaw);
	distance = pow(x-init_x, 2) + pow(y-init_y, 2);
}

void Particle::print()
{
	// printf("[x:%.2f, y:%.2f, orientation:%.3f]\n", x, y, yaw);
	if(flag){
		printf("node: \n");
	}else{
		printf("edge: %d\n", edge_num);
	}
}

void Particle::measurement_prob(double orientation)
{
	double diff = fabs(yaw-orientation);
	double fn = M_PI / 36; //5[deg]
	if(diff>M_PI) diff = 2*M_PI - diff;
	if( diff > M_PI/4 ){
		// cout << "yaw_p: " << yaw * 180 / M_PI << endl;
		// cout << "yaw_r: " << orientation * 180 / M_PI << endl;
		// weight *= 0.5 * ( 1 - diff/M_PI);
		weight = ( 1 - diff/M_PI);
		// weight *= ( 1 - diff/M_PI) * 0.1;
		// weight = 0.5;
		if(weight < 1e-5) weight = 1e-5;
	}else if(diff < fn){
		// weight *= 100*( 1 + 3 * (fn-diff)/fn);
		// weight = ( 1 + 3 * (fn-diff)/fn);
		weight = 1.0;
		// weight *= 1.5;
		if(weight > 1e5) weight = 1e5;
	}
	// weight *= weight;
}

void Particle::measurement_prob(double sense_x, double sense_y)
{
	double diff = pow(sense_x - x, 2) + pow(sense_y - y, 2);

	if(diff > pow(12, 2)){
		weight *= pow(12, 2) / diff;
	}else{
		// weight *= 1.0;
	}
}

void resampling(Particle p[], int num) //use openMP
{
	static random_device rnd;
	mt19937_64 mt(rnd());
	double sum = 0.0;
	double prob = 0.0;
	double t = 0.0;
	Particle* r;
	r = new Particle[num];

	for(int i=0; i<num; i++){
		sum += p[i].getWeight();
	}
	uniform_real_distribution<> rand(0.0, sum);
// #pragma omp parallel for private(t, prob)
	for(int j=0; j<num; j++){
		prob = 0;
		t = rand(mt);
// #pragma omp parallel for
		for(int i=0; i<num; i++){
			prob+=p[i].getWeight();
			if(t <= prob){
				r[j] = p[i];
				// cout << "i: " << i << endl;
				break;
				// flag_break = false;
			}
		}
	}
	// for(int i=0; i<num; i++){
	// 	mw = max(mw, p[i].getWeight());
	// }
	// for(int i=0; i<num; i++){
	// 	beta += rand(mt)*2.0*mw;
	// 	while(beta > p[i].getWeight()){
	// 		beta -= p[i].getWeight();
	// 		index = (index+1)%num;
	// 	}
	// 	r[i] = p[index];
	// }
// #pragma omp parallel for
	for(int i=0; i<num; i++){
		p[i] = r[i];
	}

	delete [] r;
}

