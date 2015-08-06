#include "Priority_Queue.h"
#include <math.h>


void Graph::Reverse()
{
	vertices_r.resize(vertices.size());
	for (int i = 0; i<this->vertices.size(); i++)
	{
		vertices_r[i].x = vertices[i].x;
		vertices_r[i].y = vertices[i].y;
		for (int j = 0; j<vertices[i].edges.size(); j++)
		{
			edge e;
			e = vertices[i].edges[j];
			e.id_to = i;
			vertices_r[vertices[i].edges[j].id_to].edges.push_back(e);
		}
	}
}

int Graph::Dijkstra(int s, int t, int *label, int *path)
{
	int *flag = new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());
	
	Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());

	int id = s;
	path[s] = s;
	label[s] = 0;

	//flag[s]=FINISHED;

	while (id != t)//忽略了没有路径的情况
	{
		for (int i = 0; i<vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			switch (flag[id_to])
			{
			case NEVER://从未搜索过
				label[id_to] = label[id] + vertices[id].edges[i].weight;
				Q.Push(id_to);
				flag[id_to] = EVER;
				path[id_to] = id;
				break;
			case EVER://曾经搜索过
				if (label[id] + vertices[id].edges[i].weight < label[id_to])
				{
					label[id_to] = label[id] + vertices[id].edges[i].weight;
					Q.Adjust(id_to);
					path[id_to] = id;
				}
				break;
			}
		}
		flag[id] = FINISHED;
		id = Q.Top();
		Q.Pop();

	}
	delete(flag);
	return label[t];
}

void Graph::Dijkstra_ALL(int s, hash_map<int, int> &goal,vector<int> &dist, int *label, int *path)
{
	int *flag = new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());

	Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());

	int id = s;
	path[s] = s;
	label[s] = 0;

	//flag[s]=FINISHED;
	Q.Push(id);

	while (!Q.isEmpty())//忽略了没有路径的情况
	{
		id = Q.Top();
		Q.Pop();
		auto ite = goal.find(id);
		if (ite != goal.end())
		{
			dist[ite->second] = label[id];
		}
		for (int i = 0; i<vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			switch (flag[id_to])
			{
			case NEVER://从未搜索过
				label[id_to] = label[id] + vertices[id].edges[i].weight;
				Q.Push(id_to);
				flag[id_to] = EVER;
				path[id_to] = id;
				break;
			case EVER://曾经搜索过
				if (label[id] + vertices[id].edges[i].weight < label[id_to])
				{
					label[id_to] = label[id] + vertices[id].edges[i].weight;
					Q.Adjust(id_to);
					path[id_to] = id;
				}
				break;
			}
		}
		flag[id] = FINISHED;
		
	}
	delete(flag);
	
}

void Graph::Dijkstra_ALL(int s,int *label, int *path)
{
	int *flag = new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());

	Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());

	int id = s;
	path[s] = s;
	label[s] = 0;

	//flag[s]=FINISHED;
	Q.Push(id);

	while (!Q.isEmpty())//忽略了没有路径的情况
	{
		id = Q.Top();
		Q.Pop();
		for (int i = 0; i<vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			switch (flag[id_to])
			{
			case NEVER://从未搜索过
				label[id_to] = label[id] + vertices[id].edges[i].weight;
				Q.Push(id_to);
				flag[id_to] = EVER;
				path[id_to] = id;
				break;
			case EVER://曾经搜索过
				if (label[id] + vertices[id].edges[i].weight < label[id_to])
				{
					label[id_to] = label[id] + vertices[id].edges[i].weight;
					Q.Adjust(id_to);
					path[id_to] = id;
				}
				break;
			}
		}
		flag[id] = FINISHED;

	}
	delete(flag);

}

inline int Graph::Euclidean_Dist(int a, int b)
{
	return (int)sqrt((vertices[a].x - vertices[b].x)*(vertices[a].x - vertices[b].x) +
		(vertices[a].y - vertices[b].y)*(vertices[a].y - vertices[b].y));
}

int Graph::Astar(int s, int t, int *label, int *path)//A*
{
	
	int *exlabel = new int[this->vertices.size()];//label+估值
	int *estimated_dist = new int[this->vertices.size()];//估值
	int *flag = new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());

	Priority_Queue Q(this->vertices.size() / 100, exlabel, this->vertices.size());

	int id = s;
	path[s] = s;
	label[s] = 0;

	//flag[s]=FINISHED;

	while (id != t)//忽略了没有路径的情况
	{
		/*if (id == 27153)
		{
			cout << id << endl;
			cout << vertices[id].edges.size() << endl;
		}*/
		for (int i = 0; i<vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			switch (flag[id_to])
			{
			case NEVER://从未搜索过
				label[id_to] = label[id] + vertices[id].edges[i].weight;
				estimated_dist[id_to] = Euclidean_Dist(id_to, t);
				exlabel[id_to] = label[id_to] + estimated_dist[id_to];
				Q.Push(id_to);
				flag[id_to] = EVER;
				path[id_to] = id;
				break;
			case EVER://曾经搜索过
				if (label[id] + vertices[id].edges[i].weight + estimated_dist[id_to] < exlabel[id_to])
				{
					label[id_to] = label[id] + vertices[id].edges[i].weight;
					exlabel[id_to] = label[id_to] + estimated_dist[id_to];
					Q.Adjust(id_to);
					path[id_to] = id;
				}
				break;
			}
		}
		//if(Q.Top() < 0)
		//{
		//    cout<<id<<endl;
		//	cout<<vertices[id].edges.size()<<endl;
		//}
		flag[id] = FINISHED;
		id = Q.Top();
		Q.Pop();

	}

	//this->Visualize(s,t,flag,path);

	delete(flag);
	delete(exlabel);
	delete(estimated_dist);
	return label[t];
}

int Graph::Astar_aggressive(int s, int t, int *label, int *path)//估值系数不为1的A*
{
	int *exlabel = new int[this->vertices.size()];//label+估值
	int *estimated_dist = new int[this->vertices.size()];//估值
	int *flag = new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());

	Priority_Queue Q(this->vertices.size() / 100, exlabel, this->vertices.size());

	int id = s;
	path[s] = s;
	label[s] = 0;

	flag[s] = FINISHED;

	while (id != t)//忽略了没有路径的情况
	{
		double lambda = 1.0;
		for (int i = 0; i<vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			switch (flag[id_to])
			{
			case NEVER://从未搜索过
				label[id_to] = label[id] + vertices[id].edges[i].weight;
				estimated_dist[id_to] = Euclidean_Dist(id_to, t);
				exlabel[id_to] = label[id_to] + lambda * estimated_dist[id_to];
				Q.Push(id_to);
				flag[id_to] = EVER;
				path[id_to] = id;
				break;
			case EVER://曾经搜索过
				if (label[id] + vertices[id].edges[i].weight + estimated_dist[id_to] < exlabel[id_to])
				{
					label[id_to] = label[id] + vertices[id].edges[i].weight;
					exlabel[id_to] = label[id_to] + estimated_dist[id_to];
					Q.Adjust(id_to);
					path[id_to] = id;
				}
				break;
			}
		}
		id = Q.Top();
		Q.Pop();
		flag[id] = FINISHED;
	}
	delete(flag);
	delete(exlabel);
	delete(estimated_dist);
	return label[t];
}

int Graph::Bi_direct_Astar(int s, int t, int *label, int *path)//双向A*
{
	int *exlabel = new int[this->vertices.size()];//label+估值
	int *estimated_dist = new int[this->vertices.size()];//估值
	int *flag = new int[this->vertices.size()];//搜索状态的标记
	memset(flag, 0, sizeof(int)*this->vertices.size());

	int *exlabel_r = new int[this->vertices.size()];//label+估值
	int *estimated_dist_r = new int[this->vertices.size()];//估值
	int *flag_r = new int[this->vertices.size()];//搜索状态的标记
	int *path_r = new int[this->vertices.size()];//反向path
	int *label_r = new int[this->vertices.size()];//反向label
	memset(flag_r, 0, sizeof(int)*this->vertices.size());

	for (int i = 0; i<vertices.size(); i++)
	{
		label[i] = INF;
		label_r[i] = INF;
	}

	Priority_Queue Q(this->vertices.size() / 100, exlabel, this->vertices.size());
	Priority_Queue Q_r(this->vertices.size() / 100, exlabel_r, this->vertices.size());

	int id = s;
	int id_r = t;
	path[s] = s;
	path_r[t] = t;
	label[s] = 0;
	label_r[t] = 0;

	flag[s] = FINISHED;
	flag_r[t] = FINISHED;

	int cc = 0;

	while (true)
	{

		//正向
		for (int i = 0; i<vertices[id].edges.size(); i++)
		{
			int id_to = vertices[id].edges[i].id_to;
			switch (flag[id_to])
			{
			case NEVER://从未搜索过
				label[id_to] = label[id] + vertices[id].edges[i].weight;
				estimated_dist[id_to] = Euclidean_Dist(id_to, t);
				exlabel[id_to] = label[id_to] + estimated_dist[id_to];
				Q.Push(id_to);
				flag[id_to] = EVER;
				path[id_to] = id;
				break;
			case EVER://曾经搜索过
				if (label[id] + vertices[id].edges[i].weight + estimated_dist[id_to] < exlabel[id_to])
				{
					label[id_to] = label[id] + vertices[id].edges[i].weight;
					exlabel[id_to] = label[id_to] + estimated_dist[id_to];
					Q.Adjust(id_to);
					path[id_to] = id;
				}
				break;
			}
		}
		id = Q.Top();
		Q.Pop();
		flag[id] = FINISHED;

		//check
		if (flag_r[id] == FINISHED)
			break;

		//反向
		for (int i = 0; i<vertices_r[id_r].edges.size(); i++)
		{
			int id_to = vertices_r[id_r].edges[i].id_to;
			switch (flag_r[id_to])
			{
			case NEVER://从未搜索过
				label_r[id_to] = label_r[id_r] + vertices_r[id_r].edges[i].weight;
				estimated_dist_r[id_to] = Euclidean_Dist(id_to, s);
				exlabel_r[id_to] = label_r[id_to] + estimated_dist_r[id_to];
				Q_r.Push(id_to);
				flag_r[id_to] = EVER;
				path_r[id_to] = id_r;
				break;
			case EVER://曾经搜索过
				if (label_r[id_r] + vertices_r[id_r].edges[i].weight + estimated_dist_r[id_to] < exlabel_r[id_to])
				{
					label_r[id_to] = label_r[id_r] + vertices_r[id_r].edges[i].weight;
					exlabel_r[id_to] = label_r[id_to] + estimated_dist_r[id_to];
					Q_r.Adjust(id_to);
					path_r[id_to] = id_r;
				}
				break;
			}
		}
		id_r = Q_r.Top();
		Q_r.Pop();
		flag_r[id_r] = FINISHED;

		//check
		if (flag[id_r] == FINISHED)
		{
			id = id_r;
			break;
		}
	}

	//this->Visualize_Bi(s,t,id,flag,flag_r,path,path_r);
	//PostProcessing
	int min = label[id] + label_r[id];
	//以下后处理有误差，需要再仔细考虑一下
	////再来一次正向搜索
	//for(int i=0;i<vertices[id].edges.size();i++)
	//{
	//	int id_to=vertices[id].edges[i].id_to;
	//	switch(flag[id_to])
	//	{
	//		case NEVER://从未搜索过
	//			label[id_to] = label[id] + vertices[id].edges[i].weight;
	//			estimated_dist[id_to] = Euclidean_Dist(id_to,t);
	//			exlabel[id_to]=label[id_to] + estimated_dist[id_to];
	//			Q.Push(id_to);
	//			flag[id_to]=EVER;
	//			path[id_to]=id;
	//			break;
	//		case EVER://曾经搜索过
	//			if(label[id] + vertices[id].edges[i].weight + estimated_dist[id_to] < exlabel[id_to])
	//			{
	//				label[id_to] = label[id] + vertices[id].edges[i].weight;
	//				exlabel[id_to] = label[id_to]+ estimated_dist[id_to];
	//				Q.Adjust(id_to);
	//				path[id_to]=id;
	//			}
	//			break;
	//	}
	//}

	//////找到所有横跨两个搜索空间的边，找到最小路径
	//for(Priority_Queue::Iterator iter=Q.begin();iter!=Q.end();iter++)
	//{
	//	//int a=*iter;
	//	if(label[*iter]+label_r[*iter]<min)
	//	{
	//		min=label[*iter]+label_r[*iter];
	//		if(min<0)
	//			min=min;
	//		id=*iter;
	//	}
	//}
	//this->Visualize_Bi(s,t,id,flag,flag_r,path,path_r);
	//printf("%d ",min);

	for (int i = 1; i<vertices.size(); i++)
	if (label[i] + label_r[i]<min)
	{
		min = label[i] + label_r[i];
		id = i;
	}
	//this->Visualize_Bi(s,t,id,flag,flag_r,path,path_r);
	while (id != t)
	{
		path[path_r[id]] = id;
		id = path_r[id];
	}
	this->Visualize_Bi(s, t, id, flag, flag_r, path, path_r);
	delete(flag);
	delete(exlabel);
	delete(estimated_dist);
	delete(flag_r);
	delete(exlabel_r);
	delete(estimated_dist_r);
	delete(path_r);
	delete(label_r);
	return min;
}

int Graph::Middle_vertices_search(int s, int t, int *label, int *path)//中间结点启动的Dijkstra
{
	int *path2 = new int[vertices.size()];
	int *label2 = new int[vertices.size()];
	int mid = this->FindMiddle(s, t);
	if (mid == -1)
	{
		printf("Do not find middle vertex\n");
		return this->Astar(s, t, label, path);
	}

	//int d = this->Astar(s,mid,label,path)+ this->Astar(mid,t,label2,path2);

	//并行
	int d, d2;
#pragma omp parallel num_threads(2)
	{
#pragma omp sections
		{
#pragma omp section
			d = this->Astar(s, mid, label, path);

#pragma omp section
			d2 = this->Astar(mid, t, label2, path2);
		}
	}
	d = d + d2;

	//整合两个label和path

	delete(label2);
	delete(path2);
	return d;
}
int Graph::Middle_vertices_search2(int s, int t, int *label, int *path)//中间结点启动的Dijkstra
{
	int *path2 = new int[vertices.size()];
	int *label2 = new int[vertices.size()];
	int mid = this->FindMiddle2(s, t);
	if (mid == -1)
	{
		printf("Do not find middle vertex\n");
		return this->Astar(s, t, label, path);
	}

	//int d = this->Astar(s,mid,label,path)+ this->Astar(mid,t,label2,path2);

	//并行
	int d, d2;
#pragma omp parallel num_threads(2)
	{
#pragma omp sections
		{
#pragma omp section
			d = this->Astar(s, mid, label, path);

#pragma omp section
			d2 = this->Astar(mid, t, label2, path2);
		}
	}
	d = d + d2;

	//整合两个label和path

	delete(label2);
	delete(path2);
	return d;
}
int Graph::Bi_queue_Astar(int s, int t, int *label, int *path)//双队列A*
{
	return INF;
}

void swap(int *a, int *b)
{
	int tmp = *a;
	*a = *b;
	*b = tmp;
}
void RandomSerices(int * a, int num)
{

	int i;

	for (i = 0; i < num; i++)
	{
		a[i] = i;
	}
	for (int i = num - 1; i >= 1; i--)
		swap(&a[i], &a[exRand() % (num-1)+1]);
}





void Graph::Chose_moveobj(int rate)
{
	//cout << "start chose move obj" << endl;
	int n = vertices.size();
	srand(unsigned(time(0)));
	vector<int>().swap(moveobj);
	//moveobj.clear();
	for (int i = 1; i < n; i++)
	{
		if ((rand() % 100) < rate)
		{
			moveobj.push_back(i);
		}
	}
	//cout << "end chose move obj" << endl;
}

 void Graph::Comput_Dist_Rps()
{
	 //cout << "start comput_dist_rps" << endl;
	int n = rpsentnode.size();
	//for (int i = 0; i < 100; i++)
	//	cout << rpsentnode[i] << " ";
		
	int *path = new int[vertices.size()];
	int *label=new int[vertices.size()];
	//rpsdist.clear();
	vector<vector<int>>().swap(rpsdist);
	rpsdist.resize(n);
	for (int i = 0; i != n; i++)
	{
		rpsdist[i].resize(n);
	}
	
	rpsmap.clear();
	for (int i = 0; i < n; i++)
	{
		int s = rpsentnode[i];
		rpsmap[s] = i;
	}
	/*LARGE_INTEGER begin, end, lv;
	double secondsPerTick;
	QueryPerformanceFrequency(&lv);
	secondsPerTick = 1000000.0 / lv.QuadPart;*/

	for (int i = 0; i < n; i++)
	{
		
		
		int s = rpsentnode[i];
		//QueryPerformanceCounter(&begin);
		Dijkstra_ALL(s, rpsmap, rpsdist[i], label, path);
		//QueryPerformanceCounter(&end);
		//printf("time:%f\n", secondsPerTick * (end.QuadPart - begin.QuadPart));
	}
	//cout << "end comput dist rps" << endl;
	delete(path);
	delete(label);
}


 void Graph::Comput_Dist_Rps_OMP()
 {
	 //cout << "start comput_dist_rps" << endl;
	 int n = rpsentnode.size();
	 //for (int i = 0; i < 100; i++)
	 //	cout << rpsentnode[i] << " ";

	 //rpsdist.clear();
	 vector<vector<int>>().swap(rpsdist);
	 rpsdist.resize(n);
	 for (int i = 0; i != n; i++)
	 {
		 rpsdist[i].resize(n);
	 }

	 rpsmap.clear();
	 for (int i = 0; i < n; i++)
	 {
		 int s = rpsentnode[i];
		 rpsmap[s] = i;
	 }
	 /*LARGE_INTEGER begin, end, lv;
	 double secondsPerTick;
	 QueryPerformanceFrequency(&lv);
	 secondsPerTick = 1000000.0 / lv.QuadPart;*/
	 omp_set_num_threads(2);
#pragma omp parallel for
	 for (int i = 0; i < n; i++)
	 {

		 int *path = new int[vertices.size()];
		 int *label = new int[vertices.size()];
		 int s = rpsentnode[i];
		 //QueryPerformanceCounter(&begin);
		 Dijkstra_ALL(s, rpsmap, rpsdist[i], label, path);
		 delete(path);
		 delete(label);
		 //QueryPerformanceCounter(&end);
		 //printf("time:%f\n", secondsPerTick * (end.QuadPart - begin.QuadPart));
	 }
	 //cout << "end comput dist rps" << endl;
	 
 }

 void Graph::Comput_Dist_by_Rps(int s,int *label)
 {
	 int n = vertices.size();
	 int rps1 = rpsent[s];
	 int maps = rpsmap[rps1];
	 label[s] = 0;
	 for (int i = 1; i < n; i++)
	 {

		 if (i != s)
		 {
			 int rpst = rpsent[i];
			 int mapt = rpsmap[rpst];
			 label[i] = rpsdist[maps][mapt];
		 }

	 }
 }

 void Graph::Find_KNN_Rps(int s,int k)
 {
	 //cout << "start find knn rps" << endl;
	 int rps1 = rpsent[s];
	 int maps = rpsmap[rps1];
	 int n = moveobj.size();
	 int *dist = new int[vertices.size()];
	 //Rpsknn.clear();
	 vector<int>().swap(Rpsknn);
	 Priority_Queue Q(this->vertices.size() / 100, dist, this->vertices.size());
	 
	 for (int i=0; i < n; i++)
	 {

		 int t = moveobj[i];
		 if (t != s)
		 {
			 int rpst = rpsent[t];
			 int mapt=rpsmap[rpst];
			 dist[t]=rpsdist[maps][mapt];
			 Q.Push(t);
		 }
		 
	 }
	 for (int i = 0; i < k; i++)
	 {
		 int result = Q.Top();
		 Q.Pop();
		 Rpsknn.push_back(result);
	 }
	 //cout << "end knn rps" << endl;
	 delete(dist);
 }

 void Graph::Find_KNN_Astar(int s, int k)
 {
	 //cout << "start knn astar" << endl;
	 int *dist = new int[vertices.size()];
	 int *path = new int[vertices.size()];
	 int *label = new int[vertices.size()];
	 Priority_Queue Q(this->vertices.size() / 100, dist, this->vertices.size());
	 int n = moveobj.size();
	 
	 for (int i=0; i < n; i++)
	 {
		 int t = moveobj[i];

		 if (t != s)
		 {
			
			 dist[t] = Astar(s,t,label,path);
			 Q.Push(t);
		 }
	 }
	 for (int i = 0; i < k; i++)
	 {
		 int result = Q.Top();
		 Q.Pop();
		 Astarknn.push_back(result);
	 }
	 //cout << "end knn star" << endl;
	 delete(dist);
	 delete(path);
	 delete(label);
 }

 void Graph::Find_KNN_Dijks(int s, int k)
 {
	// cout << "start knn Dijks" << endl;
	 int *dist = new int[vertices.size()];
	 int *path = new int[vertices.size()];
	 int *label = new int[vertices.size()];
	 Priority_Queue Q(this->vertices.size() / 100, dist, this->vertices.size());
	 int n = moveobj.size();
	 //dijkdist.clear();
	 vector<int>().swap(dijkdist);
	 dijkdist.resize(n);
	 dijkmap.clear();
	 for (int i = 0; i < n; i++)
	 {
		 int s = moveobj[i];
		 dijkmap[s] = i;
	 }

	 Dijkstra_ALL(s, dijkmap, dijkdist, label, path);

	 for (int i = 0; i < n; i++)
	 {
		 int t = moveobj[i];
		 dist[t] = dijkdist[i];
		 if (t != s)
		 {
			 Q.Push(t);
		 }
	 }
	 //Dijkknn.clear();
	 vector<int>().swap(Dijkknn);
	 for (int i = 0; i < k; i++)
	 {
		 int result = Q.Top();
		 Q.Pop();
		 Dijkknn.push_back(result);
	 }
	// cout << "end knn dijkstra" << endl;
	 delete(dist);
	 delete(path);
	 delete(label);
 }


 int edit(vector<int> str1, vector<int> str2)
 {
	 int max1 = str1.size();
	 int max2 = str2.size();

	 int **ptr = new int*[max1 + 1];
	 for (int i = 0; i < max1 + 1; i++)
	 {
		 ptr[i] = new int[max2 + 1];
	 }

	 for (int i = 0; i < max1 + 1; i++)
	 {
		 ptr[i][0] = i;
	 }

	 for (int i = 0; i < max2 + 1; i++)
	 {
		 ptr[0][i] = i;
	 }

	 for (int i = 1; i < max1 + 1; i++)
	 {
		 for (int j = 1; j< max2 + 1; j++)
		 {
			 int d;
			 int temp = min(ptr[i - 1][j] + 1, ptr[i][j - 1] + 1);
			 if (str1[i - 1] == str2[j - 1])
			 {
				 d = 0;
			 }
			 else
			 {
				 d = 1;
			 }
			 ptr[i][j] = min(temp, ptr[i - 1][j - 1] + d);
		 }
	 }

	
	 int dis = ptr[max1][max2];

	 for (int i = 0; i < max1 + 1; i++)
	 {
		 delete[] ptr[i];
		 ptr[i] = NULL;
	 }

	 delete[] ptr;
	 ptr = NULL;

	 return dis;
 }
 
 void Graph::SortDegrees()
 {
	 int n = vertices.size();
	 Degrees.resize(n);
	 for (int i = 0; i < n; i++)
	 {
		 Degrees[i].shapeid = i;
		 Degrees[i].degrees = vertices[i].edges.size();
	 }
	 sort(Degrees.begin(), Degrees.end());
	 //for (int i = 0; i < 100; i++)
		// cout << Degrees[i].shapeid << " " << Degrees[i].degrees << endl;
 }

 void Graph::Representative()
 {
	 int *randnodes = new int[vertices.size()];
	 RandomSerices(randnodes, vertices.size());
	 //for (int i = 0; i < 100; i++)
	 //	printf("%d ", randnodes[i]);
	 //printf("%d \n", 0x7f);
	 //printf("%d \n", INT_MAX);
	 //delete(rpsent);
	 //rpsent.clear();
	 vector<int>().swap(rpsent);
	 rpsent.resize(vertices.size());

	 int *label = new int[this->vertices.size()];
	 for (int i = 0; i < vertices.size(); i++)
		 label[i] = INF;
	 //memset(label, 0x7f, sizeof(int)*this->vertices.size());
	 //memset(rpsent, 0x7f, sizeof(int)*this->vertices.size());
	 vector<int>().swap(rpsentnode);
	 //rpsentnode.clear();
	 for (int i = 1; i < vertices.size(); ++i)
	 {

		 int randnode = randnodes[i];
		 //printf("%d \n", label[randnode]);
		 //printf("%d \n", INF);
		 //break;
		 if (label[randnode] == INF)
		 {
			 rpsentnode.push_back(randnode);
			 Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());
			 rpsent[randnode] = randnode;
			 label[randnode] = 0;
			 Q.Push(randnode);
			 while (!Q.isEmpty())
			 {
				 int v = Q.Top();
				 Q.Pop();
				 for (int i = 0; i<vertices[v].edges.size(); i++)
				 {
					 int u = vertices[v].edges[i].id_to;
					 if ((label[u]>label[v] + vertices[v].edges[i].weight) && sigma >= label[v] + vertices[v].edges[i].weight)
					 {
						 label[u] = label[v] + vertices[v].edges[i].weight;
						 Q.Push(u);
						 rpsent[u] = randnode;
					 }

				 }
			 }

		 }
	 }
	 //cout << "rpschose" << endl;
	 delete(randnodes);
	 delete(label);
 }

 void Graph::RepresentativeDegree()
 {
	 //for (int i = 0; i < 100; i++)
	 //	printf("%d ", randnodes[i]);
	 //printf("%d \n", 0x7f);
	 //printf("%d \n", INT_MAX);
	 //delete(rpsent);
	 //rpsent.clear();
	 vector<int>().swap(rpsent);
	 rpsent.resize(vertices.size());
	 SortDegrees();
	// SortDegrees();
	 int *label = new int[this->vertices.size()];
	 for (int i = 0; i < vertices.size(); i++)
		 label[i] = INF;
	 //memset(label, 0x7f, sizeof(int)*this->vertices.size());
	 //memset(rpsent, 0x7f, sizeof(int)*this->vertices.size());
	 vector<int>().swap(rpsentnode);
	 //rpsentnode.clear();
	 for (int i = 1; i < vertices.size(); ++i)
	 {

		 int randnode = Degrees[i].shapeid;
		 //printf("%d \n", label[randnode]);
		 //printf("%d \n", INF);
		 //break;
		 if (label[randnode] == INF)
		 {
			 rpsentnode.push_back(randnode);
			 Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());
			 rpsent[randnode] = randnode;
			 label[randnode] = 0;
			 Q.Push(randnode);
			 while (!Q.isEmpty())
			 {
				 int v = Q.Top();
				 Q.Pop();
				 for (int i = 0; i<vertices[v].edges.size(); i++)
				 {
					 int u = vertices[v].edges[i].id_to;
					 if ((label[u]>label[v] + vertices[v].edges[i].weight) && sigma >= label[v] + vertices[v].edges[i].weight)
					 {
						 label[u] = label[v] + vertices[v].edges[i].weight;
						 Q.Push(u);
						 rpsent[u] = randnode;
					 }

				 }
			 }

		 }
	 }
	 //cout << "rpschose" << endl;
	 delete(label);
 }

 void Graph::Representative(const vector<vertex> &vertices, const vector<int> &classes)
 {
	 int *randnodes = new int[vertices.size()];
	 RandomSerices(randnodes, vertices.size());
	 //for (int i = 0; i < 100; i++)
	 //	printf("%d ", randnodes[i]);
	 //printf("%d \n", 0x7f);
	 //printf("%d \n", INT_MAX);
	 //delete(rpsent);
	 //rpsent.clear();
	 //vector<int>().swap(rpsent);
	 rpsent.resize(this->vertices.size());
	 int myclass = classes[vertices[0].shapeid];
	 int *label = new int[this->vertices.size()];
	 for (int i = 0; i < vertices.size(); i++)
		 label[i] = INF;
	 //memset(label, 0x7f, sizeof(int)*this->vertices.size());
	 //memset(rpsent, 0x7f, sizeof(int)*this->vertices.size());
	 //vector<int>().swap(rpsentnode);//不能删除，全局代表元
	 //rpsentnode.clear();
	 for (int i = 1; i < vertices.size(); ++i)
	 {

		 int randnode = vertices[randnodes[i]].shapeid;//和原来的不同
		 //printf("%d \n", label[randnode]);
		 //printf("%d \n", INF);
		 //break;
		 if (label[randnode] == INF)
		 {
			 rpsentnode.push_back(randnode);
			 Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());
			 rpsent[randnode] = randnode;
			 Q.Push(randnode);
			 label[randnode] = 0;
			 while (!Q.isEmpty())
			 {
				 int v = Q.Top();
				 Q.Pop();
				 for (int i = 0; i<vertices[v].edges.size(); i++)
				 {
					 
					 int u = vertices[v].edges[i].id_to;
					 if (classes[u] == myclass){
						 if ((label[u]>label[v] + vertices[v].edges[i].weight) && sigma >= label[v] + vertices[v].edges[i].weight)
						 {
							 label[u] = label[v] + vertices[v].edges[i].weight;
							 Q.Push(u);
							 rpsent[u] = randnode;
						 }
					 }
				 }
			 }

		 }
	 }
	 //cout << "rpschose" << endl;
	 delete(randnodes);
	 delete(label);
 }


 void Graph::init()
 {
	 int n = vertices.size();
	 rpsent.resize(n);
	 vector<int>().swap(rpsentnode);
 }


 void Graph::Representative(const vector<int> &classes)
 {
	 int *randnodes = new int[vertices.size()];
	 RandomSerices(randnodes, vertices.size());
	 //for (int i = 0; i < 100; i++)
	 //	printf("%d ", randnodes[i]);
	 //printf("%d \n", 0x7f);
	 //printf("%d \n", INT_MAX);
	 //delete(rpsent);
	 //rpsent.clear();
	 vector<int>().swap(rpsent);
	 rpsent.resize(this->vertices.size());
	 int *label = new int[this->vertices.size()];
	 for (int i = 0; i < vertices.size(); i++)
		 label[i] = INF;
	 //memset(label, 0x7f, sizeof(int)*this->vertices.size());
	 //memset(rpsent, 0x7f, sizeof(int)*this->vertices.size());
	 vector<int>().swap(rpsentnode);
	 //rpsentnode.clear();
	 for (int i = 1; i < vertices.size(); ++i)
	 {
		 int randnode = randnodes[i];//和原来的不同
		 //printf("%d \n", label[randnode]);
		 //printf("%d \n", INF);
		 //break;
		 int myclass = classes[randnode];
		 if (label[randnode] == INF)
		 {
			 rpsentnode.push_back(randnode);
			 rpsent[randnode] = randnode;
			 Priority_Queue Q(this->vertices.size() / 100, label, this->vertices.size());
			 Q.Push(randnode);
			 label[randnode] = 0;
			 while (!Q.isEmpty())
			 {
				 int v = Q.Top();
				 Q.Pop();
				 for (int i = 0; i<vertices[v].edges.size(); i++)
				 {
					 int u = vertices[v].edges[i].id_to;
					 if (classes[u] == myclass){
						 if ((label[u]>label[v] + vertices[v].edges[i].weight) && sigma >= label[v] + vertices[v].edges[i].weight)
						 {
							 label[u] = label[v] + vertices[v].edges[i].weight;
							 Q.Push(u);
							 rpsent[u] = randnode;
						 }
					 }
				 }
			 }

		 }
	 }
	 //cout << "rpschose" << endl;
	 delete(randnodes);
	 delete(label);
 }



 void Graph::Distance_Test()
 {
	 for (int i = 0; i < 10; i++)
	 {
		 int n = this->vertices.size();
		 int randnum = exRand() % (vertices.size() - 1) + 1;
		 int *label = new int[n];
		 int *label1 = new int[n];
		 int *path = new int[n];
		// cout <<"s:"<< randnum << endl;
		 Dijkstra_ALL(randnum, label, path);
		 Comput_Dist_by_Rps(randnum, label1);
		 unsigned int dismax = 0, dismin = 10000000000, disbar = 0;
		 for (int i = 1; i < n; i++)
		 {
			 if (i != randnum)
			 {

				 unsigned int tmp = abs(label1[i] - label[i]);
				 if (dismax < tmp)
					 dismax = tmp;
				 if (dismin>tmp)
					 dismin = tmp;
				 disbar += tmp;
			 }
		 }
		 cout << "最大误差：" << dismax << "最小误差：" << dismin << "平均误差" << disbar/(n - 2) << endl;
		 delete[] label;
		 delete[] label1;
		 delete[] path;
	 }
 }