#define INF 0x3FFFFFFF
#define NEVER    0
#define EVER     1
#define FINISHED 2
#include<vector>
//需要用#define为道路分类标记和转向约束标记定义标签
#pragma  once
using namespace std;

struct center
{
	double x;
	double y;
};

struct edge//边
{
	int id_to;//指向的结点
	//zsy change int to double
	double weight;//边权值
	double max_speed;//最大通行速度
	int highway;//道路分类标记
};

struct vertex
{
	//20150412 zsy add index
	//20150412 zsy change int to double (x,y)
	int shapeid;
	//int index;
	vector<edge> edges;//邻边
	double x;//x坐标
	double y;//y坐标
	int turn_restr;//转向约束标记
};

struct obstacle//多边形障碍
{
	vector<double> x;//多边形顶点的x坐标
	vector<double> y;//多边形顶点的y坐标
};
