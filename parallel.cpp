// compilation: mpiCC program name.cpp
//run: mpirun ./a.out



#define _CRT_SECURE_NO_WARNINGS

#include<iostream>
#include<algorithm>
#include<stdio.h>
#include<vector>
#include <mpi.h>
#include<cstring>
using namespace std;


#define starting 1
#define ending 2
#define distance 3
#define rank 4
#define bsize 5
#define cdist 6

const bool SUCCESS = true;
int boardSize = 3;
int startingboard[100];
int endingboard[100];

class state {

public:
	int b[10][10];
	int g, f;

	state *P;

	state() {
		g = 0;
		f = 0;
		P = NULL;
	}

	static int hdistance(state from, state to) {
		int i, j, dist = 0;
		for (i = 0; i < boardSize; i++)
			for (j = 0; j < boardSize; j++)
				if (from.b[i][j] != to.b[i][j])
					dist++;
		return dist;
	}

	bool operator ==(state a) {
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++)
				if (this->b[i][j] != a.b[i][j])
					return false;
		return true;
	}

	void disp() {
		for (int i = 0; i < boardSize; i++) {
			for (int j = 0; j < boardSize; j++)
				cout << b[i][j] << " ";
			cout << endl;
		}
		printf("g: %d\n", g);
		printf("f: %d\n\n", f);
	}
};

vector<state>PPath;


bool esolution(int *s, int d)
{
	int i, j, xy = 0, r, ni = 0;
	int si = d*d;
	bool sol = false;

	int *nb;
	nb = (int*)malloc(sizeof(si));
	memcpy(nb, s, si);

	for (i = 0; i < si; i++) {
		if (nb[i] == 0) {
			xy = i;
		}
	}

	r = xy / d;

	for (i = xy; i < si - 1; i++) {
		nb[i] = nb[i + 1];
	}

	for (i = 0; i < si - 2; i++) {
		for (j = i + 1; j < si - 1; j++) {
			if (nb[i] > nb[j]) {
				ni++;
			}
		}
	}

	if (d % 2 == 0) {
		if ((r + ni) % 2 != 0) {
			sol = true;
		}
	}
	else {
		if (ni % 2 == 0) {
			sol = true;
		}
	}

	return sol;
}

bool esolution(state t, int n) {
	int i, y, z, k, w, m = 0, j, x;
	bool s = true;
	for (i = 0; i < n; i++)
		for (y = 0; y < n; y++)
		{
			if (y != n - 1) { z = y + 1; w = i; }
			else { z = 0; w = i + 1; }
			for (k = w; k < n; k++)
			{

				for (j = z; j < n; j++)
				{
					z = 0;
					if ((t.b[i][y] > t.b[k][j]) && (t.b[i][y] != 0) && (t.b[k][j] != 0))
						m++;
				}
			}
		}
	if (n % 2 == 1)
	{
		if (m % 2 == 1)	
			s = false;
	}
	else				
	{
		for (i = 0; i < n; i++)
			for (j = 0; j < n; j++)
				if (t.b[i][j] == 0) x = i;

		if ((m + x) % 2 == 0)
			s = false;
	}
	return s;
}

bool Fvalue(state x, state y) {
	return x.f < y.f;
}

void twist(state &x, int s, int t, int pi, int pj) {
	int xs;
	xs= x.b[s][t];
	x.b[s][t] = x.b[pi][pj];
	x.b[pi][pj] = xs;
}

bool Check(state a, vector<state> b) {
	for (int i = 0; i < b.size(); i++)
		if (a == b[i])
			return true;

	return false;
}

void siblings(state c, state end, int ni, int nj, int pi, int pj,
	vector<state> &os, vector<state> cs) {

	state ns = c;
	twist(ns, ni, nj, pi, pj);
	if (!Check(ns, cs)) {

		int totalg = c.g + 1;

		if (!Check(ns, os) || totalg < ns.g) {

			ns.g = totalg;
			ns.f = ns.g + state::hdistance(ns, end);

			state *t = new state();
			*t = c;
			ns.P = t;
			os.push_back(ns);
		}
	}
}

void sib(state c, state g, vector<state> &os, vector<state> cs) {
	int i, j, pi, pj;

	//Find the pition of '0'
	for (i = 0; i < boardSize; i++)
		for (j = 0; j < boardSize; j++)
			if (c.b[i][j] == 0)
			{
				pi = i;
				pj = j;
			}

	i = pi; j = pj;

	if ((i - 1) >= 0) {
		siblings(c, g, (i - 1), j, pi, pj, os, cs);
	}

	if ((i + 1) < boardSize) {
		siblings(c, g, (i + 1), j, pi, pj, os, cs);
	}

	if ((j - 1) >= 0) {
		siblings(c, g, i, (j - 1), pi, pj, os, cs);
	}

	if ((j + 1) < boardSize) {
		siblings(c, g, i, (j + 1), pi, pj, os, cs);
	}
}

void path(state c, vector<state> &P) {
	state *t = &c;
	while (t != NULL) {
		P.push_back(*t);
		t = t->P;
	}
}

bool Astar(state s, state end, vector<state> &ot) {
	vector<state> cs;
	vector<state> os;

	state c;

	s.g = 0;
	s.f = s.g + state::hdistance(s, end);

	os.push_back(s);
	int y = 0;
	while (!os.empty()) {

		sort(os.begin(), os.end(), Fvalue);

		c = os[0]; 

		if (c == end) {
			path(c, ot);
			return SUCCESS;
		}

		os.erase(os.begin());
		cs.push_back(c);

		sib(c, end, os, cs);
	}

	return !SUCCESS;
}

bool Correct(state c, int ni, int nj, int pi, int pj,
	vector<state> &os, vector<state> cs) {

	state ns = c;
	twist(ns, ni, nj, pi, pj);
	if (!Check(ns, cs)) {

		int totalg = c.g + 1;

		if (!Check(ns, os) || totalg < ns.g) {
			return true;
		}
	}
	return false;
}

int See(state c, vector<state> os, vector<state> cs) {
	int m = 0;
	int i, j, pi, pj;

	for (i = 0; i < boardSize; i++)
		for (j = 0; j < boardSize; j++)
			if (c.b[i][j] == 0)
			{
				pi = i;
				pj = j;
			}

	i = pi; j = pj;

	if ((i - 1) >= 0) {
		if (Correct(c, (i - 1), j, pi, pj, os, cs))
			m++;
	}

	if ((i + 1) < boardSize) {
		if (Correct(c, (i + 1), j, pi, pj, os, cs))
			m++;
	}

	if ((j - 1) >= 0) {
		if (Correct(c, i, (j - 1), pi, pj, os, cs))
			m++;
	}

	if ((j + 1) < boardSize) {
		if (Correct(c, i, (j + 1), pi, pj, os, cs))
			m++;
	}
	return m;
}

int Parallel(state s, state end, int size) {
	vector<state> cs;
	vector<state> os;
	vector<state> otp;
	state c;

	s.g = 0;
	s.f = s.g + state::hdistance(s, end);

	os.push_back(s);

	while (!os.empty()) {

		sort(os.begin(), os.end(), Fvalue);

		c = os[0]; 

		if (c == end) {
			path(c, otp);
			for (int i = otp.size() - 1; i >= 0; i--) {
				otp[i].disp();
			}
			cout << "\nnumber of moves: " << otp.size() - 1 << endl;
			return -1;
		}
		if ((os.size() - 1 + See(c, os, cs)) <= size - 1)
		{
			os.erase(os.begin());
			cs.push_back(c);
			sib(c, end, os, cs);
		}
		else
		{
			cout << " os size is " << os.size() << "\n";
			fflush(stdout);
			break;
		}
	}
	for (int s = 1; s <= os.size(); s++)
	{
		int h = 0;
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++) {
				startingboard[h] = os[s - 1].b[i][j];
				h++;
			}
		int g = os[s - 1].g;
		MPI_Send(&boardSize, 1, MPI_INT, s, bsize, MPI_COMM_WORLD);
		MPI_Send(&g, 1, MPI_INT, s, cdist, MPI_COMM_WORLD);
		MPI_Send(startingboard, boardSize*boardSize, MPI_INT, s, starting, MPI_COMM_WORLD);
		MPI_Send(endingboard, boardSize*boardSize, MPI_INT, s, ending, MPI_COMM_WORLD);
        //exit(0);
    }
	for (int s = os.size(); s <size; s++)
	{
		int babo = -1;
		MPI_Send(&babo, 1, MPI_INT, s, bsize, MPI_COMM_WORLD);
	}
	return os.size();
}

double start_time, finish_time, total_time;

void Sequential(int rk, int size) {
	MPI_Status status;

	MPI_Recv(&boardSize, 1, MPI_INT, 0, bsize, MPI_COMM_WORLD, &status);
	if (boardSize != -1) {
		int g = 1;
		MPI_Recv(&g, 1, MPI_INT, 0, cdist, MPI_COMM_WORLD, &status);
		MPI_Recv(startingboard, boardSize*boardSize, MPI_INT, 0, starting, MPI_COMM_WORLD, &status);
		MPI_Recv(endingboard, boardSize*boardSize, MPI_INT, 0, ending, MPI_COMM_WORLD, &status);
		int h = 0;
		state st, end;
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++) {
				st.b[i][j] = startingboard[h];
				end.b[i][j] = endingboard[h];
				h++;
			}
		vector<state>ott;
        start_time = MPI_Wtime();
		if (Astar(st, end, ott) == SUCCESS) {
			for (int i = ott.size() - 1; i >= 0; i--) {
				ott[i].disp();
			}
			cout << "\nNumber of moves: " << ott.size() - 1 << endl;
            finish_time = MPI_Wtime();
		total_time = finish_time - start_time;
		cout<< "\nTotal time taken:"<<total_time<<endl;
			cout << "The Rank Is " << rk << endl;
            //exit(0);
			int a = ott.size() - 1 + g;
			MPI_Send(&a, 1, MPI_INT, 0, distance, MPI_COMM_WORLD);
			MPI_Send(&rk, 1, MPI_INT, 0, rank, MPI_COMM_WORLD);

		}
		else
			cout << "\nunsucessful.\n";
	}


}

int main(int argc, char *argv[]) {
	int rk, size;
    //double start_time, finish_time, total_time;

	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &rk);
	MPI_Comm_size(MPI_COMM_WORLD, &size);

	if (rk == 0)
	{
	
		cout<<"\nEnter the start board";
        for(int i=0;i<9;i++)
            cin>>startingboard[i];

        cout<<"\nEnter the goal board";
        for(int i=0;i<9;i++)
            cin>>endingboard[i];
            

		boardSize = 3;

		int h = 0;
		state start, goal;
		for (int i = 0; i < boardSize; i++)
			for (int j = 0; j < boardSize; j++) {
				start.b[i][j] = startingboard[h];
				goal.b[i][j] = endingboard[h];
				h++;
			}
        //start_time = MPI_Wtime();

		if (esolution(start, boardSize))
		{

			int c = Parallel(start, goal, size);
			if (c == -1)
			{
				cout << " the master solve the problem ";
				for (int i = 1; i < size; i++)
				{
					int babo = -1;
					MPI_Send(&babo, 1, MPI_INT, i, bsize, MPI_COMM_WORLD);
				}
			}
			else {
				int recivedStates[100][2];
				for (int i = 1; i <= c; i++) {
					int min = 0, pRank = 0;
					MPI_Status status;
					MPI_Recv(&min, 1, MPI_INT, i, distance, MPI_COMM_WORLD, &status);
					MPI_Recv(&pRank, 1, MPI_INT, i, rank, MPI_COMM_WORLD, &status);
					recivedStates[i - 1][0] = pRank;
					recivedStates[i - 1][1] = min;
				}
					
			}
		}
		else
		{
			cout << "This Board Is UnSolveable";
			for (int i = 1; i < size; i++)
			{
				int babo = -1;
				MPI_Send(&babo, 1, MPI_INT, i, bsize, MPI_COMM_WORLD);
			}
		}
        /*finish_time = MPI_Wtime();
		total_time = finish_time - start_time;
		cout<< "\nTotal time taken:"<<total_time;*/

	}
   
	else
	{
		Sequential(rk, size);
	}

	MPI_Finalize();
	return 0;
}