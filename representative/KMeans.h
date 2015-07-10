#include"node.h"
#include <vector>
#include <queue>
#include <limits>
#include<hash_map>
#include<iostream>
//#include"Graph.h"
#include<random>
using namespace std;


class KMeans
{
public:
	int k;
	double mu;
	vector<center> centers;//´ØÖÐÐÄ
	vector<int> classes;
	KMeans()
	{

	}
	KMeans(int k1, double mu1) :k(k1), mu(mu1)
	{
		centers.resize(k);
	}
	void initCenter(double xLeft, double xRight, double yUp, double yDown);

	void classify(const vector<vertex> &vertices);

	bool calNewCenter(const vector<vertex> &vertices);

	double getSati(const vector<vertex> &vertices, int len);

	double run(const vector<vertex> &vertices, double xLeft, double xRight, double yUp, double yDown);

};
