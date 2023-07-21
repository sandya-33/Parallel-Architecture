
#define _CRT_SECURE_NO_WARNINGS

#include<iostream>
#include<algorithm>
#include<stdio.h>
#include<vector>
#include <cstring>
#include <sys/time.h>
using namespace std;

#define SIZ 3

const bool SUCCESS = true;

class state {

public:
	int board[SIZ][SIZ];
	int g_val, f_val;

	state *previous;

	state() {
		g_val = 0;
		f_val = 0;
		previous = NULL;
	}

	static int heuristic(state from, state to) {
		int i, j, distance = 0;
		for (i = 0; i<SIZ; i++)
			for (j = 0; j<SIZ; j++)
				if (from.board[i][j] != to.board[i][j])
					distance++;
		return distance;
	}

	bool operator ==(state a) {
		for (int i = 0; i<SIZ; i++)
			for (int j = 0; j<SIZ; j++)
				if (this->board[i][j] != a.board[i][j])
					return false;
		return true;
	}

	void print() {
		for (int i = 0; i<SIZ; i++) {
			for (int j = 0; j<SIZ; j++)
				cout << board[i][j] << " ";
			cout << endl;
		}
		printf("g: %d\n", g_val);
		printf("f:: %d\n\n", f_val);
	}
};

vector<state> op;


bool isSolvable(int *board, int dimen)
{
	int i, j, zeroind=0, blRow, inver=0;
	int size = dimen*dimen;
	bool solvable = false;

	int *n_Board;
	n_Board = (int*)malloc(sizeof(size));
	memcpy(n_Board, board, size);


	for (i = 0; i<size; i++) {
		if (n_Board[i] == 0) {
			zeroind = i;
		}
	}

	blRow = zeroind / dimen;

	for (i = zeroind; i<size - 1; i++) {
		n_Board[i] = n_Board[i + 1];
	}

	for (i = 0; i<size - 2; i++) {
		for (j = i + 1; j<size - 1; j++) {
			if (n_Board[i] > n_Board[j]) {
				inver++;
			}
		}
	}

	if (dimen % 2 == 0) {
		if ((blRow + inver) % 2 != 0) {
			solvable = true;
		}
	}
	else {
		if (inver % 2 == 0) {
			solvable = true;
		}
	}

	return solvable;
}

bool lowerF(state a, state b) {
	return a.f_val < b.f_val;
}

void swap(state &a, int m, int n, int posx, int posy) {
	int temp;
	temp = a.board[m][n];
	a.board[m][n] = a.board[posx][posy];
	a.board[posx][posy] = temp;
}

bool isInSet(state a, vector<state> b) {
	for (int i = 0; i<b.size(); i++)
		if (a == b[i])
			return true;

	return false;
}

void addNeighbor(state present, state goal, int newI, int newJ, int posi, int posj,
	vector<state> &openlst, vector<state> closedlst) {

	state newstate = present;
	swap(newstate, newI, newJ, posi, posj);
	if (!isInSet(newstate, closedlst)) {

		int tentative_g_score = present.g_val + 1;

		if (!isInSet(newstate, openlst) || tentative_g_score < newstate.g_val) {

			newstate.g_val = tentative_g_score;
			newstate.f_val = newstate.g_val + state::heuristic(newstate, goal);

			state *temp = new state();
			*temp = present;
			newstate.previous = temp;
			openlst.push_back(newstate);
		}
	}
}

void findneighbors(state present, state goal, vector<state> &openlst, vector<state> closedlst) {
	int i, j, posi, posj;

	for (i = 0; i<SIZ; i++)
		for (j = 0; j<SIZ; j++)
			if (present.board[i][j] == 0)
			{
				posi = i;
				posj = j;
			}

	i = posi; j = posj;

	if ((i - 1) >= 0) {
		addNeighbor(present, goal, (i - 1), j, posi, posj, openlst, closedlst);
	}

	if ((i + 1)<SIZ) {
		addNeighbor(present, goal, (i + 1), j, posi, posj, openlst, closedlst);
	}

	if ((j - 1) >= 0) {
		addNeighbor(present, goal, i, (j - 1), posi, posj, openlst, closedlst);
	}

	if ((j + 1)<SIZ) {
		addNeighbor(present, goal, i, (j + 1), posi, posj, openlst, closedlst);
	}
}

void reconstr_path(state present, vector<state> &previous) {
	state *temp = &present;
	while (temp != NULL) {
		previous.push_back(*temp);
		temp = temp->previous;
	}
}

bool astar(state start, state goal) {
	vector<state> closedlst;
	vector<state> openlst;

	state present;

	start.g_val = 0;
	start.f_val = start.g_val + state::heuristic(start, goal);

	openlst.push_back(start);

	while (!openlst.empty()) {

		sort(openlst.begin(), openlst.end(), lowerF);

		present = openlst[0];

		if (present == goal) {
			reconstr_path(present, op);
			return SUCCESS;
		}

		openlst.erase(openlst.begin());
		closedlst.push_back(present);

		findneighbors(present, goal, openlst, closedlst);
	}

	return !SUCCESS;
}



int main(int argc, char *argv[]) {
	

	state start, goal;
	int i, j;
    struct timeval begin, end;
    

	cout << "Enter " << SIZ << "x" << SIZ << " board :\n";
	for (i = 0; i<SIZ; i++)
		for (j = 0; j<SIZ; j++)
			cin >> start.board[i][j];

	cout << "Enter goal board :\n";
	for (i = 0; i<SIZ; i++)
		for (j = 0; j<SIZ; j++)
			cin >> goal.board[i][j];
    gettimeofday(&begin, 0);
	

	if (astar(start, goal) == SUCCESS) {
		for (i = op.size() - 1; i >= 0; i--) {
			op[i].print();
		}
		cout << "\nNumber of moves: " << op.size() - 1 << endl;
        gettimeofday(&end, 0);
    long seconds = end.tv_sec - begin.tv_sec;
    long microseconds = end.tv_usec - begin.tv_usec;
    double elapsed = seconds + microseconds*1e-6;
    
    printf("Time measured: %.3f seconds.\n", elapsed);
	}
	else
		cout << "\n unsuccessful.\n";
	
	return 0;
}
