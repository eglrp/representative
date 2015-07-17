/************
#pragma   once
************/
#pragma  once
#include "CImg.h"
using namespace cimg_library;
#include <vector>
#include <queue>
#include <limits>
#include<hash_map>
#include"node.h"
#include<omp.h>
//#include <algorithm>
using namespace std;



#include "shapelib-1.3.0\shapefil.h"
#include <iostream>
#include <fstream>
#include "tools.h"
class Graph
{
private://私有成员自行添加，不要删除原有内容。

	vector<vertex> vertices_r;//反向图
	obstacle obstacles;//障碍集合
	vector<vector<vector<int>>> Grids;
	//坐标的四个边界
	//double bound_west;
	//double bound_east;
	//double bound_north;
	//double bound_south;

	double sigma;//精度

	bool isDist;//边权值是否为距离
	int global_max_speed;//全局最大通行速度
	//------zsy add 20150416 tiger only
	double global_max_weight;//最大边权值
	//--------------------------------

	void Visualize(int s, int t, int *flag, int *path);//搜索空间的可视化
	void Visualize_Bi(int s, int t, int mid, int *flag, int *flag2, int *path, int *path2);//双向搜索空间的可视化
	void map_point(int v, int w, int h, int &x, int &y);//将图上的一个点v映射到w*h的图片上的一个像素(x,y)
	int FindNearst(int x, int y);//给定坐标，查找一个可能比较近的结点。失败返回-1
	int FindMiddle(int s, int t);//找到s-t的某个中间点
	int FindMiddle2(int s, int t);
	//-------zsy add 20150413--------
	void LoadPolyLine(SHPHandle, int, double*, double*);
	void LoadPoint(SHPHandle, int, double*, double*);
    //void LoadDb(DBFHandle handle);
	void UpdateBound(vertex);//更新边界
	void UpdateWeightBound(edge);//更新权值上界
	//-------------------------------
	void Swap(int a, int b);
	void RandomNodes();//节点随机化
	

public:
	double bound_west;
	double bound_east;
	double bound_north;
	double bound_south;
	vector<vertex> vertices;//图的结点集，从1开始，0处为无效值
	vector<int> moveobj;//移动对象
	vector<int> classes;//分块类别信息
	vector<degree> Degrees;//节点度排序


	vector<vector<int>> rpsdist;//代表元之间的距离矩阵
	hash_map<int, int> rpsmap;//代表元和距离矩阵的映射
	hash_map<int, int> dijkmap;
	vector<int> dijkdist;
	//int *rpsent;
	vector<int> rpsent;//每个节点的代表元
	vector<int> Rpsknn;//代表元knn结果
	vector<int> Astarknn;//Astarknn结果
	vector<int> Dijkknn;//dijkstraknn结果
	vector<int> rpsentnode;//代表节点


	void init();

	void SetSigma(double sig);//设置sigma
	double GetSigma();
	int grid_size;//网格边长
	void Reverse();//反转
	void LoadGraph_shp(char *dir);//读入图数据
	void LoadGraph_gr(char *file_gr, char *file_co);//读入图数据
	void GridsGen(int size);//划分为边长为size的网格
	void Convert();//消除约束
	int Dijkstra(int s, int t, int *label, int *path);
	void Dijkstra_ALL(int s, hash_map<int, int> &goal, vector<int> &dist, int *label, int *path);//全源最短路径dijkstra
	void Comput_Dist_Rps();//计算代表元之间的距离
	void Chose_moveobj(int rate);//按概率选取移动对象
	
	void Find_KNN_Rps(int s, int k);//通过代表元找KNN
	void Find_KNN_Astar(int s, int k);//通过astar找knn
	void Find_KNN_Dijks(int s, int k);//通过dijkstra找knn

	void SortDegrees();//对节点度排序
	void Representative();//选取代表元
	void RepresentativeDegree();
	void Representative(const vector<vertex> &vertices, const vector<int> &classes);//
	void Representative(const vector<int> &classes);


	int DFS(int,int *);//DFS搜索，返回连通点数量
	int Astar(int s, int t, int *label, int *path);//A*
	int Astar_aggressive(int s, int t, int *label, int *path);//估值系数不为1的A*
	int Bi_direct_Astar(int s, int t, int *label, int *path);//双向A*
	int Bi_queue_Astar(int s, int t, int *label, int *path);//双队列A*
	int Euclidean_Dist(int a, int b);//结点A和结点B的直线距离
	int Middle_vertices_search(int s, int t, int *label, int *path);//中间结点启动的Dijkstra
	//===========zsy add 2050412==============
	int Middle_vertices_search2(int s, int t, int *label, int *path);
	void LoadGraph_shp(char *dir,const char *filename);//读入指定的shp文件
	void LoadGraph_tiger(char * file_tiger);//读入tiger数据
	void ResetGraph();//重置图中所有数据
	void ExportConnectedGraph(int start, const char * grfile);
	//========================================
};


/*******************
调用处理shape文件的库
将dir文件夹下的若干文件中包含的数据读入到Graph对应的成员中：
vector<vertex> vertices;
obstacle obstacles;
int bound_west;
int bound_east;
int bound_north;
int bound_south;
bool isDist;
int global_max_speed;

若障碍多边形数据存在问题，则自行随机生成一些。
*******************/
void Graph::SetSigma(double sig)
{
	sigma = sig;
}

double Graph::GetSigma()
{
	return sigma;
}
//void Graph::LoadGraph_shp(char *dir)
//{
//	vector<string> files;
//	get_filenames(dir,files);
//	vector<string>::iterator iter;
//	
//	for(iter = files.begin();iter != files.end();iter++)
//	{
//		LoadGraph_shp(dir,(*iter).c_str());
//	}
//	return;
//}
inline double get_dist(double a, double b)
{
	return sqrt(a*a + b*b);
}
/************************
个人认为参数应该是shp文件名，因为指定了SHP文件之后，对应的dbf和其他文件都应该是同名的
同时，一个文件夹下可能存在多个shp文件表示多个图层
************************/
//void Graph::LoadGraph_shp(char *dir, const char *filename)
//{
//	char * file = new char[strlen(dir) + strlen(filename) + 1];
//	file = strcpy(file,dir);
//	file = strcat(file,filename);
//	file = strcat(file,".shp");
//	SHPHandle handle = SHPOpen(file,"rb");
//	int pnEntities,pnShapeType;//entity count and shape type
//    double padfMinBound[4], padfMaxBound[4];//
//	SHPGetInfo(handle,&pnEntities,&pnShapeType,padfMinBound,padfMaxBound);//get file info
//	std::cout<<"loading "<<file<<"("<<pnEntities<<"entities)"<<std::endl;
//	switch(pnShapeType)
//	{
//	case SHPT_POINT:
//		//LoadPoint(handle,pnEntities,padfMinBound,padfMaxBound);
//		break;
//	case SHPT_ARC:
//		//LoadPolyLine(handle,pnEntities,padfMinBound,padfMaxBound);
//		break;
//	}
//	SHPClose(handle);
//}
/**********************
读入tiger数据
在tiger数据中坐标为经纬度
我将经度(longitude)设置为x
纬度(latitude)设置为y
整型
小数部分为无用信息
**********************/
void Graph::LoadGraph_tiger(char * file_tiger)
{
	//TODO : not sure whether need this
	//ResetGraph();
	std::ifstream infile(file_tiger);
	if (!infile)
	{
		printf("Error: cannot open the file: %s\n", file_tiger);
		return;
	}
	int size = 0;
	string read;
	vector<string> contents;
	vertex v;
	vertices.push_back(v);
	//read node
	{//initialize bound
		getline(infile, read);
		size = atoi(read.c_str());
		cout << "graph has " << size << " vertices\n";
		cout << "loading verticies\n";
		getline(infile, read);
		split(read, " ", contents);
		vertex v;
		v.x = atof(contents[1].c_str());
		v.y = atof(contents[2].c_str());
		bound_west = bound_east = v.x;
		bound_north = bound_south = v.y;
		vertices.push_back(v);
	}
	for (int i = 2; i <= size; i++)
	{
		contents.clear();
		//cout<<"reading "<<i<<"/"<<size<<"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		getline(infile, read);
		split(read, " ", contents);
		vertex v;
		v.x = atof(contents[1].c_str());
		v.y = atof(contents[2].c_str());
		vertices.push_back(v);
		UpdateBound(v);
	}
	//read edge
	int idfrom, idto;
	double length;
	double time;
	{
		getline(infile, read);
		size = atoi(read.c_str());
		cout << "\ngraph has " << size << " edges\n";
		cout << "loading edges\n";
		contents.clear();
		getline(infile, read);//read id
		split(read, " ", contents);
		edge e1, e2;
		idto = atoi(contents[1].c_str()) + 1;
		idfrom = atoi(contents[0].c_str()) + 1;
		e1.id_to = idto;
		e2.id_to = idfrom;
		contents.clear();

		getline(infile, read);//read attr
		split(read, " ", contents);
		time = atof(contents[0].c_str());
		length = atof(contents[1].c_str());
		e1.max_speed = e2.max_speed = length / time;
		e1.highway = e2.highway = atoi(contents[2].c_str());
		e1.weight = e2.weight = length;
		vertices[idfrom].edges.push_back(e1);
		vertices[idto].edges.push_back(e2);
		global_max_speed = e1.max_speed;
		global_max_weight = length;
	}
	for (int i = 2; i <= size; i++)
	{
		//cout<<"reading "<<i<<"/"<<size<<"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
		contents.clear();
		getline(infile, read);//read id
		split(read, " ", contents);
		edge e1, e2;
		idto = atoi(contents[1].c_str()) + 1;
		idfrom = atoi(contents[0].c_str()) + 1;
		e1.id_to = idto;
		e2.id_to = idfrom;
		contents.clear();

		getline(infile, read);//read attr
		split(read, " ", contents);
		time = atof(contents[0].c_str());
		length = atof(contents[1].c_str());
		e1.max_speed = e2.max_speed = length / time;
		e1.highway = e2.highway = atoi(contents[2].c_str());
		e1.weight = e2.weight = length;
		vertices[idfrom].edges.push_back(e1);
		vertices[idto].edges.push_back(e2);
		UpdateWeightBound(e1);
	}
	cout << "Finish loading graph data\n";
	infile.close();
}
void Graph::LoadGraph_gr(char *file_gr, char *file_co)//读入图数据
{
	int *maxdegree;
	double sum_arc = 0;
	FILE *fp = fopen(file_gr, "r");
	if (!fp)
	{
		printf("Error: cannot open the file: %s\n", file_gr);
		return;
	}
	char buf[200] = { 0 };
	int size = 0, n_arc = 0;
	//skip some information
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fscanf(fp, "p sp %d%d\n", &size, &n_arc);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	printf("This graph has %d vertices\n", size);
	printf("%d edges\n", n_arc);
	size++;
	//nodes=(vertex*)malloc(sizeof(vertex)*(size));
	vertices.resize(size);
	//memset(nodes,0,sizeof(vertex)*size);

	int from, to, weight;
	edge a;
	for (int i = 0; i<n_arc; i++)
	{
		char c = fgetc(fp);
		c = fgetc(fp);
		fscanf(fp, "%d%d%d", &from, &a.id_to, &a.weight);
		sum_arc += a.weight;

		//nodes[from].id=from;
		vertices[from].edges.push_back(a);
		//nodes[from].degree++;
	}
	fclose(fp);
	int cc[30] = { 0 };
	int max = 0;
	for (int i = 1; i<size; i++)
	{
		if (vertices[i].edges.size()>max)
			max = vertices[i].edges.size();
		cc[vertices[i].edges.size()]++;
	}
	printf("Finish loading graph data\n");
	//if(max>15)
	//	printf("Warning: the max degree of vertex in this graph is more than 15. SPLZ cannot calculate a correct result.\n");

	fp = fopen(file_co, "r");
	if (!fp)
	{
		printf("Error: cannot open the file: %s\n", file_co);
		return;
	}

	//skip some information
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	fgets(buf, 200, fp);
	int v;
	double x, y;
	this->bound_east = -INF;
	this->bound_west = INF;
	this->bound_north = -INF;
	this->bound_south = INF;
	for (int i = 1; i<size; i++)
	{
		char c = fgetc(fp);
		c = fgetc(fp);
		fscanf(fp, "%d%lf%lf", &v, &x, &y);
		vertices[v].shapeid = v;
		vertices[v].x = x;
		vertices[v].y = y;
		UpdateBound(vertices[v]);
	}
	fclose(fp);
}

void Graph::map_point(int v, int w, int h, int &x, int &y)
{
	x = (int)(1.0*w*(vertices[v].x - bound_west) / (bound_east - bound_west));
	y = (int)(1.0*h*(vertices[v].y - bound_south) / (bound_north - bound_south));
}

void draw_point(CImg<unsigned char> &img, int x, int y, int r, unsigned char c[3])
{
	for (int a = -r; a <= r; a++)
	for (int b = -r; b <= r; b++)
	{
		int xx = x + a, yy = y + b;
		if (xx<0)
			xx = 0;
		if (xx >= img.width())
			xx = img.width() - 1;
		if (yy<0)
			yy = 0;
		if (yy >= img.height())
			yy = img.height() - 1;
		img(xx, yy, 0, 0) = c[0];
		img(xx, yy, 0, 1) = c[1];
		img(xx, yy, 0, 2) = c[2];
	}
}

void Graph::GridsGen(int size)//划分为边长为(bound_east-bound_west)/size+1的网格
{
	grid_size = (bound_east - bound_west) / size + 1;
	int w = abs(bound_east - bound_west) / grid_size + 1;
	int h = abs(bound_north - bound_south) / grid_size + 1;
	Grids.resize(h);
	classes.resize(vertices.size());
	for (int i = 0; i<Grids.size(); i++)
		Grids[i].resize(w);
	for (int i = 1; i<vertices.size(); i++)
	{
		int x = (vertices[i].x - bound_west) / grid_size;
		int y = (vertices[i].y - bound_south) / grid_size;
		classes[i] = y*w + x;
		Grids[y][x].push_back(i);
	}

	int c = 0;
	for (int i = 0; i<h; i++)
	for (int j = 0; j<w; j++)
	if (Grids[i][j].size()>0)
		c++;
	printf("%d grids is not empty\n", c);
}

int Graph::FindNearst(int x, int y)//给定坐标，查找一个可能比较近的结点。失败返回-1
{
	int grid_x = (x - bound_west) / grid_size;
	int grid_y = (y - bound_south) / grid_size;
	int min = INF;
	int min_v = -1;
	for (int i = 0; i<Grids[grid_y][grid_x].size(); i++)
	{
		int dist = sqrt((vertices[Grids[grid_y][grid_x][i]].x - x)*(vertices[Grids[grid_y][grid_x][i]].x - x)
			+ (vertices[Grids[grid_y][grid_x][i]].y - y)*(vertices[Grids[grid_y][grid_x][i]].y - y));
		if (dist<min)
		{
			min = dist;
			min_v = Grids[grid_y][grid_x][i];
		}
	}
	return min_v;
}
int Graph::FindMiddle(int s, int t)//找到s-t的某个中间点
{
	//return FindNearst((vertices[s].x+vertices[t].x)/2,(vertices[s].y+vertices[t].y)/2);
	int x = ((vertices[s].x + vertices[t].x) / 2 - bound_west) / grid_size;
	int y = ((vertices[s].y + vertices[t].y) / 2 - bound_south) / grid_size;
	if (Grids[y][x].size())
	{
		int r = rand() % Grids[y][x].size();
		return Grids[y][x][r];
	}
	else
		return -1;
}

int Graph::FindMiddle2(int s, int t)//找到s-t的某个中间点
{
	//return FindNearst((vertices[s].x+vertices[t].x)/2,(vertices[s].y+vertices[t].y)/2);

	double slope = 0;
	if ((vertices[t].x - vertices[s].x) == 0)
	{
		if ((vertices[t].y - vertices[s].y) > 0)
			slope = 10;
		else
			slope = -10;
	}
	else{
		slope = (vertices[t].y - vertices[s].y) / (vertices[t].x - vertices[s].x);
	}
	int x = ((vertices[s].x + vertices[t].x) / 2 - bound_west) / grid_size;
	int y = ((vertices[s].y + vertices[t].y) / 2 - bound_south) / grid_size;
	int x1 = x;
	int y1 = y;
	int *path = new int[vertices.size()];
	int *path2 = new int[vertices.size()];
	int *label = new int[vertices.size()];
	int *label2 = new int[vertices.size()];

	if (slope > -1 / 2 && slope <= 1 / 2)
	{
		x1 = x + 1;
	}
	else if (slope > 1 / 2 && slope <= 3 / 2)
	{
		x1 = x + 1;
		y1 = y + 1;
	}
	else if (slope > 3 / 2)
	{
		y1 = y + 1;
	}
	else if (slope <= -1 / 2 && slope > -3 / 2)
	{
		x1 = x - 1;
		y1 = y + 1;
	}
	else if (slope <= -3 / 2)
	{
		x1 = x + 1;
	}
	if (x1 >= 0 && y1 >= 0)
	{
		if (Grids[y][x].size())
		{
			int s1 = rand() % Grids[y][x].size();
			int s2 = rand() % Grids[y][x].size();
			if (Grids[y1][x1].size())
			{
				int t1 = rand() % Grids[y1][x1].size();
				int t2 = rand() % Grids[y1][x1].size();

				int node1 = Grids[y][x][s1];
				int node2 = Grids[y][x][s2];
				int node3 = Grids[y1][x1][t1];
				int node4 = Grids[y1][x1][t2];
				/*int node1 = 52912;
				int node2 = 53518;
				int node3 = Grids[y1][x1][t1];
				int node4 = Grids[y1][x1][t2];*/
				if (s1 != s2&&t1 != t2)
				{

					Astar(node1, node3, label, path);
					Astar(node2, node4, label2, path2);
					printf("x:%d y:%d\n", node1, node3);
					printf("x1:%d y1:%d\n", node2, node4);
					//int t = path[Grids[y1][x1][t1]];
					//int s = path2[Grids[y1][x1][t2]];

					//printf("%d \n", node1);
					//printf("%d\n", node2);
					//printf("%d\n", node3);
					//printf("%d\n", node4);
					/*for (int i = node3; i != node1;)
					{
					//printf("%d ", i);
					i = path[i];
					printf("%d ", i);
					}
					printf("\n");
					for (int j = node4; j != node2;)
					{
					//printf("%d ", i);
					j = path2[j];
					printf("%d ", j);
					}
					printf("\n");*/
					for (int i = node3; i != node1;)
					{
						for (int j = node4; j != node2;)
						{
							if (j == i)
							{
								delete(path);
								delete(path2);
								delete(label);
								delete(label2);
								return i;
							}
							else
								j = path2[j];
						}
						i = path[i];
					}
					Astar(node1, node4, label, path);
					Astar(node2, node3, label2, path2);

					for (int i = node4; i != node1;)
					{
						for (int j = node3; j != node2;)
						{
							if (j == i)
							{
								delete(path);
								delete(path2);
								delete(label);
								delete(label2);

								return i;
							}
							else
								j = path2[j];
						}
						i = path[i];
					}
				}
				else
				{
					delete(path);
					delete(path2);
					delete(label);
					delete(label2);

					return Grids[y][x][s1];
				}
			}
			else
			{
				delete(path);
				delete(path2);
				delete(label);
				delete(label2);
				return Grids[y][x][s1];
			}
		}
		else
		{
			delete(path);
			delete(path2);
			delete(label);
			delete(label2);

			return -1;
		}
	}
	else
	{
		delete(path);
		delete(path2);
		delete(label);
		delete(label2);

		return -1;
	}
}
void Graph::Visualize(int s, int t, int *flag, int *path)
{
	int w = 1000;
	int h = 1.0*w*abs(bound_north - bound_south) / abs(bound_east - bound_west);
	CImg<unsigned char> img(w, h, 1, 3);
	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };
	unsigned char yellow[3] = { 255, 255, 0 };
	unsigned char black[3] = { 0, 0, 0 };
	img.fill(255);
	int x, y;
	//画出搜索空间
	for (int i = 1; i<this->vertices.size(); i++)
	{
		map_point(i, w, h, x, y);
		if (flag[i] == FINISHED)
			draw_point(img, x, y, 1, red);
	}
	for (int i = 1; i<this->vertices.size(); i++)
	{
		map_point(i, w, h, x, y);
		if (flag[i] == EVER)
			draw_point(img, x, y, 1, yellow);
	}
	//画出路径
	for (int i = t; i != s; i = path[i])
	{
		map_point(i, w, h, x, y);
		draw_point(img, x, y, 1, black);
	}
	//画出起始点
	map_point(s, w, h, x, y);
	draw_point(img, x, y, 3, black);
	map_point(t, w, h, x, y);
	draw_point(img, x, y, 3, black);

	img.display("search space");
}

void Graph::Visualize_Bi(int s, int t, int mid, int *flag, int *flag2, int *path, int *path2)
{
	int w = 1000;
	int h = 1.0*w*abs(bound_north - bound_south) / abs(bound_east - bound_west);
	CImg<unsigned char> img(w, h, 1, 3);
	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };
	unsigned char yellow[3] = { 255, 255, 0 };
	unsigned char black[3] = { 0, 0, 0 };
	img.fill(255);
	int x, y;
	//画出搜索空间
	for (int i = 1; i<this->vertices.size(); i++)
	{
		map_point(i, w, h, x, y);
		if (flag[i] == FINISHED)
			draw_point(img, x, y, 1, red);
		if (flag2[i] == FINISHED)
			draw_point(img, x, y, 1, blue);
	}
	for (int i = 1; i<this->vertices.size(); i++)
	{
		map_point(i, w, h, x, y);
		if (flag[i] == EVER)
			draw_point(img, x, y, 1, yellow);
		if (flag2[i] == EVER)
			draw_point(img, x, y, 1, green);
	}
	//画出路径
	for (int i = mid; i != s; i = path[i])
	{
		map_point(i, w, h, x, y);
		draw_point(img, x, y, 1, black);
	}
	for (int i = mid; i != t; i = path2[i])
	{
		map_point(i, w, h, x, y);
		draw_point(img, x, y, 1, black);
	}
	//画出起始点
	map_point(s, w, h, x, y);
	draw_point(img, x, y, 3, black);
	map_point(t, w, h, x, y);
	draw_point(img, x, y, 3, black);

	img.display("search space");
}
//void Graph::LoadPoint(SHPHandle handle,int pnEntities, 
//	double* padfMinBound, double* padfMaxBound)
//{
//	std::cout<<"loading point:\n";
//	std::cout<<"resizing vector:\n";
//	vertices.resize(pnEntities);
//	for(int i = 0;i < pnEntities;i++)
//	{
//		std::cout<<i + 1<<"/"<<pnEntities<<"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
//		SHPObject *o = SHPReadObject(handle,i);
//		
//		    vertex v;
//			v.x = o->padfX[0];
//			v.y = o->padfY[0];
//			//v.index = vertices.size();
//			v.shapeid = o->nShapeId;
//			vertices.push_back(v);
//		
//		SHPDestroyObject(o);
//	}
//	std::cout<<std::endl;
//}
//void Graph::LoadPolyLine(SHPHandle handle,int pnEntities,
//	double* padfMinBound, double* padfMaxBound)
//{
//	std::cout<<"loading polyline:\n";
//	for(int i = 0;i < pnEntities;i++)
//	{
//		std::cout<<i + 1<<"/"<<pnEntities<<"\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b";
//		SHPObject *o = SHPReadObject(handle,i);
//		//polyline only
//		if(o->nVertices == 1)
//		{		
//			std::cout<<"polyline with single point"<<std::endl;
//		    /*vertex v;
//			v.x = o->padfX[0];
//			v.y = o->padfY[0];
//			v.index = vertices.size();
//			v.shapeid = o->nShapeId;
//			vertices.push_back(v);*/
//		}else
//		{
//			vertex v1,v2;
//			edge e;
//			v1 = vertices[o->panPartStart[0]];
//			v2 = vertices[o->panPartStart[o->nParts - 1]];
//			//TODO: how to find the end point
//			e.id_to = v2.shapeid;
//			e.weight = 0;
//			for(int i = 1;i < o->nVertices;i++)
//			{
//				e.weight += get_dist(o->padfX[i - 1] - o->padfX[i],o->padfY[i - 1] - o->padfY[i]);
//			}
//			v1.edges.push_back(e);
//		}
//		SHPDestroyObject(o);
//	}
//	std::cout<<std::endl;
//}
void Graph::UpdateBound(vertex v)
{
	if (v.x < bound_west)
		bound_west = v.x;
	else if (v.x > bound_east)
		bound_east = v.x;
	if (v.y < bound_south)
		bound_south = v.y;
	else if (v.y > bound_north)
		bound_north = v.y;
}
void Graph::UpdateWeightBound(edge e)
{
	if (e.max_speed > global_max_speed)
		global_max_speed = e.max_speed;
	if (e.weight > global_max_weight)
		global_max_weight = e.weight;
}
void Graph::ResetGraph()
{
	vertices.clear();
	vertices_r.clear();;//反向图
	obstacles.x.clear();//障碍集合
	obstacles.y.clear();

	//坐标的四个边界
	double bound_west = (numeric_limits<double>::min)();
	double bound_east = (numeric_limits<double>::max)();
	double bound_north = (numeric_limits<double>::max)();
	double bound_south = (numeric_limits<double>::min)();

	bool isDist = true;//边权值是否为距离
	int global_max_speed = 0;//全局最大通行速度
	//------zsy add 20150416 tiger only
	double global_max_weight = 0;//最大边权值
}

int Graph::DFS(int start, int * flag)
{
	//int *flag=new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());
	int count = 0;
	queue<int> v;
	v.push(start);
	int id = start;
	while (id > 0)//
	{
		for (int i = 0; i < vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			if (flag[id_to] == NEVER)
			{
				v.push(id_to);
				count++;
				flag[id_to] = EVER;
			}
		}
		flag[id] = FINISHED;
		if (v.empty())
			id = -1;
		else
		{
			id = v.front();
			v.pop();
		}
	}
	//delete(flag);
	return count;
}
void Graph::ExportConnectedGraph(int start, const char * tiger)
{
	int *flag = new int[this->vertices.size()];//backward map
	int count = DFS(start, flag);
	//int *map = new int[count + 2];//file to graph
	//int *map2 = new int[this->vertices.size()];
	int index = 1;
	for (int i = 1; i < this->vertices.size(); i++)
	{
		if (flag[i] == FINISHED)
		{
			//map[index] = i;
			flag[i] = index;
			index++;
		}
		else
			flag[i] = -1;
	}
	cout << "last index:" << index - 1 << endl;
	cout << "count:" << count << endl;
	FILE * grout;
	FILE * arctmp;
	string tmp = string(tiger) + ".tiger";
	grout = fopen(tiger, "w+");
	arctmp = fopen(tmp.c_str(), "w+");
	fprintf(grout, "%d\n", count);
	int arccount = 0;
	for (int i = 1; i < this->vertices.size(); i++)
	{
		if (flag[i] < 0)
			continue;
		//write node
		vertex v = this->vertices[i];
		fprintf(grout, "%d %d %d\n", flag[i] - 1, v.x, v.y);
		//write edge tmp
		for (int j = 0; j < v.edges.size(); j++)
		{
			edge e = v.edges[j];
			int idto = e.id_to;
			if (idto > i && flag[idto] > 0)
			{
				fprintf(arctmp, "%d %d\n", flag[i] - 1, flag[idto] - 1);
				fprintf(arctmp, "%lf %lf %d\n", e.weight / e.max_speed, e.weight, e.highway);
				arccount++;
			}
		}
	}
	fclose(arctmp);
	arctmp = fopen(tmp.c_str(), "r");
	fprintf(grout, "%d\n", arccount);
	char buffer[100];
	for (int i = 0; i <arccount; i++)
	{
		fgets(buffer, 100, arctmp);
		fputs(buffer, grout);
		fgets(buffer, 100, arctmp);
		fputs(buffer, grout);
	}
	fclose(grout);
	fclose(arctmp);
	remove(tmp.c_str());
	delete(flag);
	//delete(map);
	//delete[] buffer;
}

#include"Astar.h"
//#include "Astar.h"