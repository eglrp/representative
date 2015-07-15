
#include"KMeans.h"
using namespace std;
void KMeans::initCenter(double xLeft, double xRight, double yUp, double yDown) {
	default_random_engine e;
	uniform_real_distribution<double> u(0, 1);
	double xSize = (xRight - xLeft);
	double ySize = (yUp - yDown);
	//cout << "xleft" <<fixed<<xLeft << "xright" << xRight << "yup" << yUp << "ydown" << yDown << endl;
	//cout << "k" << k << "mu" << mu << endl;
	for (int i = 0; i < k; i++){
		double x = xLeft + u(e)* xSize;
		double y = yDown + u(e)* ySize;
		center cen;
		cen.x = x;
		cen.y = y;
		centers[i]=cen;
		// System.out.println(x+" and "+y);
	}
}
int times = 0;
void KMeans::classify(const vector<vertex> &vertices) {
	//cout << "调用" << ++times << endl;
	int n = vertices.size();
	for (int i = 1; i != n; i++)
	{
		int index = -1;
		int neardist = INF;
		double x = vertices[i].x;
		double y = vertices[i].y;
		for (int i = 0; i < k; i++)
		{
			double dist = sqrt((x - centers[i].x)*(x - centers[i].x) + (y - centers[i].y)*(y - centers[i].y));
			//  System.out.println(i+": "+dist) ;
			// 计算距离
			if (dist < neardist) {
				neardist = dist;
				index = i;
			}
		}
		classes[i] = index;
	}
}

bool KMeans::calNewCenter(const vector<vertex> &vertices) {
	
	int len = 2;
	bool end = true;
	vector<int> count;
	count.resize(k);
	//int count = new int[k]; // 记录每个簇有多少个元素
	vector<vector<double>>sum;
	sum.resize(k);
	//double[][] sum = new double[k][];
	for (int i = 0; i < k; i++)
		sum[i].resize(len);
	int n = vertices.size();
	for (int i = 1; i != n; i++)
	{

		int id = classes[i];
		count[id]++;
		sum[id][0] += vertices[i].x;
		sum[id][1] += vertices[i].y;
	}
	for (int i = 0; i < k; i++) {
		if (count[i] != 0) {
			for (int j = 0; j < len; j++) {
				sum[i][j] /= count[i];
			}
		}
		// 簇中不包含任何点,及时调整质心
		else {
			int a = (i + 1) % k;
			int b = (i + 3) % k;
			int c = (i + 5) % k;

			centers[i].x = (centers[a].x + centers[b].x + centers[c].x) / 3;
			centers[i].y = (centers[a].y + centers[b].y + centers[c].y) / 3;
		}
	}
	for (int i = 0; i < k; i++) {
		if (count[i] != 0){
			// 只要有一个质心需要移动的距离超过了mu，就返回false
			// if (Global.calEuraDist(sum[i], center[i], len) >= mu) { //使用欧氏距离
			double dist = sqrt((sum[i][0] - centers[i].x)*(sum[i][0] - centers[i].x) + (sum[i][1] - centers[i].y)*(sum[i][1] - centers[i].y));
			if (dist >= mu) { // 使用编辑距离
				//       System.out.println(dist) ;
				end = false;
				break;
			}
		}
	}
	if (!end) {
		for (int i = 0; i < k; i++) {

			centers[i].x = sum[i][0];
			centers[i].y = sum[i][1];
		}
	}
	return end;
}


double KMeans::getSati(const vector<vertex> &vertices, int len) {
	double satisfy = 0.0;
	int n = vertices.size();
	vector<int> count;
	count.resize(k);
	vector<double> ss;
	ss.resize(k);
	//double[] ss = new double[k];
	//Iterator<Node> iter = nodes.iterator();
	for (int i = 1; i < n; i++)
	{
		int id = classes[i];
		count[id]++;

		ss[id] += pow(vertices[i].x - centers[id].x, 2.0);
		ss[id] += pow(vertices[i].y - centers[id].y, 2.0);
	}
	for (int i = 0; i < k; i++) {
		satisfy += count[i] * ss[i];
	}
	return satisfy;
}


double KMeans::run(const vector<vertex> &vertices, double xLeft, double xRight, double yUp, double yDown) {

	//cout <<"k" <<k <<"mu" <<mu << endl;
	int n = vertices.size();
	classes.resize(n);
	initCenter(xLeft, xRight, yUp, yDown);
	classify(vertices);
	while (!calNewCenter(vertices)) {
		//    System.out.println("again") ;
		classify(vertices);
	}
	//for (int i = 0; i < k; i++)
	//	cout << "x:" << centers[i].x << " y:" << centers[i].y << endl;
	//double ss = getSati(vertices, 2);
	//printf("加权方差：%f", ss);
	return 0;
}