
#ifndef LOCALIZATION_PARTICLE_H
#define LOCALIZATION_PARTICLE_H

// #include<random>
// extern class std::random_device;
// extern class std::mt19937_64;
// extern class std::normal_distribution;

class Particle{
	double x;
	double y;
	double yaw; //[rad]
	double weight;
	// double forward_noise;
	// double mu;
	// static std::random_device rnd;
	// std::mt19937_64 mt;
	// std::normal_distribution<> error;
	bool flag_fast;
	double init_x, init_y;
	int edge_num;
	double distance; //sqared of dist [m]

	// void setGauss();
	// double getError();

	public:
	bool flag;
	double x_new;
	double y_new;
	double dist_r_last;
	int cnt_yaw;
	int edge_last;

	Particle();
	Particle(double, double, double, double, double); //x, y, yaw, mu, noise
	// void randSet(int);
	void setXY(double, double);
	void setWeight(double);
	// void setNoise(double);
	// void setMu(double);
	void setInit(double, double);
	void setEdge(int);
	void setFast(bool);
	double getX();
	double getY();
	double getYaw();
	double getWeight();
	double getEdge();
	double getDist();
	double distSquared(double, double); // x, y
	void sense();
	void move(double, double); //distance, direction
	void measurement_prob(double); // orientation
	void measurement_prob(double, double); // x, y
	void print(); // [x: , y: , orientation: ]
};

void resampling(Particle [], int); //use openMP

#endif

