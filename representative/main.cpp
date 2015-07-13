
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <time.h>
#include <omp.h>
#include "Graph.h"
#include "tools.h"
#include"KMeans.h"
int spy=0;

int main()
{
	
	LARGE_INTEGER begin,end,lv;
	double secondsPerTick;
	QueryPerformanceFrequency(&lv);
	secondsPerTick=1000000.0/lv.QuadPart;
	srand(time(0));

	
	////a.push_back(99);
	//a.push_back(100);
	//b.push_back(101);
	//b.push_back(102);
	//b.push_back(100);
	//cout<<edit(a, b) << endl;

	Graph graph;
	graph.LoadGraph_gr("E:\\gis\\USA-road-d.Ny.gr","E:\\gis\\USA-road-d.ny.co");
	//graph.LoadGraph_tiger("E:\\GIS\\CT.tmp");

	//修正gr中的数据误差
	//for(int i=1;i<graph.vertices.size();i++)
	//	for(int j=0;j<graph.vertices[i].edges.size();j++)
	//		if(graph.Euclidean_Dist(i,graph.vertices[i].edges[j].id_to)>graph.vertices[i].edges[j].weight)
	//			//printf("Wrong graph data %d %d\n",graph.Euclidean_Dist(i,graph.vertices[i].edges[j].id_to),graph.vertices[i].edges[j].weight);
	//			graph.vertices[i].edges[j].weight=graph.Euclidean_Dist(i,graph.vertices[i].edges[j].id_to);
	
	
	//int *flag = new int[graph.vertices.size()];
	//int c = graph.DFS(1,flag);
	//cout<<"dfs count "<<c<<"/"<<graph.vertices.size() - 1<<endl;
	//if(c < graph.vertices.size() - 1)
	//{
	//	cout<<"handling new data"<<endl;
	//	graph.ExportConnectedGraph(1,"E:\\GIS\\CT.tmp.tiger");
	//	//memset(flag,0,graph.vertices.size()*sizeof(int));
	//	graph.ResetGraph();
	//	graph.LoadGraph_tiger("E:\\GIS\\CT.tmp.tiger");
	//	cout<<"new graph data dfs result:"<<graph.DFS(1,flag)<<"/"<<graph.vertices.size() - 1<<endl;
	//}
	//delete[] flag;
	double totaltime = 0, totaltime1=0,totaltime2 = 0, totaltime3 = 0;
	graph.Reverse();
	for (int i = 0; i < 100; i++)
		cout << graph.vertices[i].shapeid << " ";
	graph.GridsGen(100);
	cout << sizeof(int) << endl;
	cout << graph.bound_north << " " << graph.bound_south << " " << graph.bound_east;
	KMeans kmeans(100,100);
	
	kmeans.run(graph.vertices, graph.bound_west, graph.bound_east, graph.bound_north, graph.bound_south);


	
	/*int rpsnum;//代表元生成测试
	double rpstime;
	for (int i = 0; i < 12; i++)
	{
		rpsnum = 0;
		rpstime = 0;
		graph.SetSigma(400000 + i * 2000);
		for (int j = 0; j < 100; j++)
		{
			QueryPerformanceCounter(&begin);
			graph.Representative();
			QueryPerformanceCounter(&end);
			rpstime += secondsPerTick * (end.QuadPart - begin.QuadPart);
			//printf("representative  node num: %d", graph.rpsentnode.size());
			rpsnum += graph.rpsentnode.size();
		}
		printf("representative  node num: %f,sigma: %f,time:%fus\n", rpsnum/100.0, graph.GetSigma(),rpstime/100);
	}
	*/
	/*double precom1, precom2, precom3;//测试不同误差上限的运行时间
	for (int i = 0; i < 4; i++)
	{
		precom1 = 0;
		precom2 = 0;
		graph.SetSigma(10000 + i * 5000);

		QueryPerformanceCounter(&begin);
		graph.Representative();
		QueryPerformanceCounter(&end);
		precom1 = secondsPerTick * (end.QuadPart - begin.QuadPart);
		printf("representative  node num: %d,sigma: %f,time:%fus\n", graph.rpsentnode.size(), graph.GetSigma(), precom1);

		QueryPerformanceCounter(&begin);
		graph.Comput_Dist_Rps();
		QueryPerformanceCounter(&end);
		precom2 = secondsPerTick * (end.QuadPart - begin.QuadPart);
		printf("representative  node num: %f,sigma: %f,time:%fus\n", graph.rpsentnode.size(), graph.GetSigma(), precom1);
		printf("precompute cost time:%fus\n", precom2);
		printf("all precompute cost time:%fus\n", precom1 + precom2);
	}
	*/

	/*double precom1, precom2, precom3;//测试预处理时间
	for (int i = 0; i < 4; i++)
	{
		precom1 = precom2 = 0;
		graph.SetSigma(5000 + i * 5000);

		QueryPerformanceCounter(&begin);
		graph.Representative();
		QueryPerformanceCounter(&end);
		precom1 = secondsPerTick * (end.QuadPart - begin.QuadPart);
		printf("representative  node num: %d,sigma: %f,time:%fus\n", graph.rpsentnode.size(), graph.GetSigma(), precom1);

		QueryPerformanceCounter(&begin);
		graph.Comput_Dist_Rps();
		QueryPerformanceCounter(&end);
		precom2 = secondsPerTick * (end.QuadPart - begin.QuadPart);


		printf("precompute cost time:%fus\n", precom2);
		printf("all precompute cost time:%fus\n", precom1 + precom2);
	}
	*/

	
	/*graph.SetSigma(10000);

	printf("%f\n", graph.GetSigma());

	QueryPerformanceCounter(&begin);
	graph.Representative();//选择代表元
	QueryPerformanceCounter(&end);
	totaltime = secondsPerTick * (end.QuadPart - begin.QuadPart);

	printf("representative  node num %d,representative time %f\n", graph.rpsentnode.size(),totaltime);

	QueryPerformanceCounter(&begin);
	graph.Comput_Dist_Rps();//预处理计算代表元之间的距离
	QueryPerformanceCounter(&end);
	totaltime1 = secondsPerTick * (end.QuadPart - begin.QuadPart);
	printf("comput dist rps time %f\n", totaltime1);
	*/

	//graph.Chose_moveobj(1);
	//cout << "move obj num:" << graph.moveobj.size() << endl;

	//QueryPerformanceCounter(&begin);
	//graph.Find_KNN_Rps(graph.moveobj[0], 20);
	//QueryPerformanceCounter(&end);
	//totaltime2 = secondsPerTick * (end.QuadPart - begin.QuadPart);
	//printf("find knn by rps cost time %f\n", totaltime2);


	//QueryPerformanceCounter(&begin);
	//graph.Find_KNN_Dijks(graph.moveobj[0], 100);
	//QueryPerformanceCounter(&end);
	//totaltime3 = secondsPerTick * (end.QuadPart - begin.QuadPart);
	//printf("find knn by Dijkstra cost time %f\n", totaltime3);

	//for (auto key : graph.Rpsknn)
	//{
	//	cout << key<<" ";
	//}
	//cout << endl;
	//int *path = new int[graph.vertices.size()];
	////int *path2=new int[graph.vertices.size()];
	//int *label = new int[graph.vertices.size()];
	//for (auto key : graph.Dijkknn)
	//{
	//	cout << key<<" ";
	//	//cout << graph.dijkdist[graph.dijkmap[key]] << endl;
	//	//cout << graph.Dijkstra(100, key, label, path) << endl;
	//}
	
	//cout << graph.Dijkstra(100, 4798, label, path) << endl;
	
	/*cout << endl;
	int rate=edit(graph.Rpsknn, graph.Dijkknn);
	cout <<"rate:"<<rate<< endl;*/
/*
	srand(unsigned(time(0)));
	for (int i = 0; i < 10; i++)
	{
		graph.Chose_moveobj(2+i*2);//按比例选择移动对象
		cout << "move obj num:" << graph.moveobj.size() << endl;
		int ratebar = 0;
		int num = 0;
		for (int j = 0; j < 100; j++)
		{
			int s=rand() % graph.moveobj.size();
			graph.Find_KNN_Rps(s, 10);
			graph.Find_KNN_Dijks(s, 10);
			int nums = 0;
			for (int i = 0; i < 10; i++)
			{
				for (int j = 0; j < 10; j++)
				{
					if (graph.Rpsknn[i] == graph.Dijkknn[j])
					{
						nums++;
						break;
					}
				}
			}
			num += nums;
			int rate = 10-edit(graph.Rpsknn, graph.Dijkknn);
			ratebar += rate;
		}
		cout << "move obj rate num:" << 2+2*i <<"right rate"<<ratebar/100.0<<"nums same:"<<num/100.0<< endl;
	}


	graph.SetSigma(5000);

	printf("%f\n", graph.GetSigma());

	QueryPerformanceCounter(&begin);
	graph.Representative();//选择代表元
	QueryPerformanceCounter(&end);
	totaltime = secondsPerTick * (end.QuadPart - begin.QuadPart);

	printf("representative  node num %d,representative time %f\n", graph.rpsentnode.size(), totaltime);

	QueryPerformanceCounter(&begin);
	graph.Comput_Dist_Rps();//预处理计算代表元之间的距离
	QueryPerformanceCounter(&end);
	totaltime1 = secondsPerTick * (end.QuadPart - begin.QuadPart);
	printf("comput dist rps time %f\n", totaltime1);

	srand(unsigned(time(0)));
	for (int i = 0; i < 10; i++)
	{
		graph.Chose_moveobj(2 + i * 2);//按比例选择移动对象
		cout<< "move obj num:" << graph.moveobj.size() << endl;
		int ratebar = 0;
		int num = 0;
		for (int j = 0; j < 100; j++)
		{
			int s = rand() % graph.moveobj.size();
			graph.Find_KNN_Rps(s, 10);
			graph.Find_KNN_Dijks(s, 10);
			int nums = 0;
			for (int i = 0; i < 10; i++)
			{
				for (int j = 0; j < 10; j++)
				{
					if (graph.Rpsknn[i] == graph.Dijkknn[j])
					{
						nums++;
						break;
					}
				}
			}
			num += nums;
			int rate = 10 - edit(graph.Rpsknn, graph.Dijkknn);
			ratebar += rate;
		}
		cout << "move obj rate num:" << 2 + 2 * i << "right rate" << ratebar / 100.0 << "nums same:" << num / 100.0 << endl;
	}

	cout << "k=100" << endl;
	for (int i = 0; i < 10; i++)
	{
		graph.Chose_moveobj(2 + i * 2);//按比例选择移动对象
		cout << "move obj num:" << graph.moveobj.size() << endl;
		int ratebar = 0;
		int num = 0;
		for (int j = 0; j < 100; j++)
		{
			int s = rand() % graph.moveobj.size();
			graph.Find_KNN_Rps(s, 100);
			graph.Find_KNN_Dijks(s, 100);
			int nums = 0;
			for (int i = 0; i < 100; i++)
			{
				for (int j = 0; j < 100; j++)
				{
					if (graph.Rpsknn[i] == graph.Dijkknn[j])
					{
						nums++;
						break;
					}
				}
			}
			num += nums;
			int rate = 100 - edit(graph.Rpsknn, graph.Dijkknn);
			ratebar += rate;
		}
		cout << "move obj rate num:" << 2 + 2 * i << "right rate" << ratebar / 100.0 << "nums same:" << num / 100.0 << endl;
	}

	*/
	//
	//int *path=new int[graph.vertices.size()];
	//int *path2=new int[graph.vertices.size()];
	//int *label=new int[graph.vertices.size()];
	//int *label2=new int[graph.vertices.size()];

	//
	//double error=0,error1=0;

	//
	//int N=1000;//测试次数
	//double totaltime=0,totaltime2=0,totaltime3=0;
	//for (int i = 0; i<N; i++)
	//{
	//	int s = exRand() % (graph.vertices.size() - 1) + 1;
	//	int t = exRand() % (graph.vertices.size() - 1) + 1;
	//	//s = 213086;
	//	//t = 529756;

	//	QueryPerformanceCounter(&begin);
	//	//int dist=graph.Astar(s,t,path,label);
	//	printf("node s:%d ", s);
	//	printf("node t:%d \n", t);
	//	int dist = graph.Astar(s, t, path, label);
	//	QueryPerformanceCounter(&end);
	//	totaltime += secondsPerTick * (end.QuadPart - begin.QuadPart);

	//	printf("node s:%d ", s);
	//	printf("node t:%d \n", t);
	//	QueryPerformanceCounter(&begin);
	//	int dist2 = graph.Middle_vertices_search(s, t, path2, label2);
	//	QueryPerformanceCounter(&end);
	//	totaltime2 += secondsPerTick * (end.QuadPart - begin.QuadPart);
	//	printf("node s:%d ", s);
	//	printf("node t:%d \n", t);
	//	QueryPerformanceCounter(&begin);
	//	int dist3 = graph.Middle_vertices_search2(s, t, path2, label2);
	//	QueryPerformanceCounter(&end);
	//	totaltime3 += secondsPerTick * (end.QuadPart - begin.QuadPart);

	//	error += (dist2 - dist)*1.0 / dist;
	//	error1 += (dist3 - dist)*1.0 / dist;
	//	//if(dist!=dist2)
	//	//	printf("Wrong answer! %d %d\n",dist,dist2);

	//}
	//
	//printf("Cost time1=%fus\n",totaltime/N);
	//printf("Cost time2=%fus\n",totaltime2/N);
	//printf("Cost time2=%fus\n", totaltime3/ N);
	//printf("Error=%.2f%%\n",error/N*100);
	//printf("Error1=%.2f%%\n", error1 / N * 100);
	//delete(label);
	//delete(path);
	//delete(path2);
	//delete(label2);
	//return 0;
}