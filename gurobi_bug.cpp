// gurobi bug.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include<iostream>
#include<algorithm>
#include<bitset>
#include<vector>
#include<string>
#include<array>
#include<map>
#include"gurobi_c++.h"

using namespace std;

const int MAX = 200000000;

void triviumCore(GRBModel& model, vector<GRBVar>& x, int i1, int i2, int i3, int i4, int i5)
{
    GRBVar y1 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y2 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y3 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y4 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar y5 = model.addVar(0, 1, 0, GRB_BINARY);

    GRBVar z1 = model.addVar(0, 1, 0, GRB_BINARY);
    GRBVar z2 = model.addVar(0, 1, 0, GRB_BINARY);

    // z3 and z4 are not needed, since z3 = z4 = a
    GRBVar a = model.addVar(0, 1, 0, GRB_BINARY);

    //copy
    model.addConstr(y1 <= x[i1]);
    model.addConstr(z1 <= x[i1]);
    model.addConstr(y1 + z1 >= x[i1]);

    //copy
    model.addConstr(y2 <= x[i2]);
    model.addConstr(z2 <= x[i2]);
    model.addConstr(y2 + z2 >= x[i2]);

    //copy
    model.addConstr(y3 <= x[i3]);
    model.addConstr(a <= x[i3]);
    model.addConstr(y3 + a >= x[i3]);

    //copy
    model.addConstr(y4 <= x[i4]);
    model.addConstr(a <= x[i4]);
    model.addConstr(y4 + a >= x[i4]);
    //XOR
    model.addConstr(y5 == x[i5] + a + z1 + z2);

    x[i1] = y1;
    x[i2] = y2;
    x[i3] = y3;
    x[i4] = y4;
    x[i5] = y5;
}


void MidSolutionCounter(int rounds, const bitset<80>& cube, const
    bitset<288>& last, float
    time, int thread,int mipFocus)
{
    GRBEnv env = GRBEnv();
    env.set(GRB_IntParam_Threads, thread);
    env.set(GRB_IntParam_PoolSearchMode, 2);//focus on finding n best solutions
    env.set(GRB_IntParam_MIPFocus, mipFocus);
    env.set(GRB_IntParam_PoolSolutions, MAX); // try to find 2000000
    GRBModel model = GRBModel(env);

    vector<GRBVar> s(288);
    for (int i = 0; i < 288; i++)
        s[i] = model.addVar(0, 1, 0, GRB_BINARY);

    for (int i = 0; i < 80; i++)
        if (cube[i] == 0)
            model.addConstr(s[i + 93] == 0);
        else
            model.addConstr(s[i + 93] == 1);

    // key, last three bits
    for (int i = 80; i < 93; i++)
        model.addConstr(s[i] == 0);
    for (int i = 93 + 80; i < 285; i++)
        model.addConstr(s[i] == 0);

    vector<GRBVar> works = s;
    for (int r = 0; r < rounds; r++)
    {
        triviumCore(model, works, 65, 170, 90, 91, 92);
        triviumCore(model, works, 161, 263, 174, 175, 176);
        triviumCore(model, works, 242, 68, 285, 286, 287);

        vector<GRBVar> temp = works;
        for (int i = 0; i < 288; i++)
            works[(i + 1) % 288] = temp[i];
    }

    for (int i = 0; i < 288; i++)
        if (last[i] == 1)
            model.addConstr(works[i] == 1);
        else
            model.addConstr(works[i] == 0);

    GRBLinExpr nk = 0;
    for (int i = 0; i < 80; i++)
        nk += s[i];
    model.setObjective(nk, GRB_MAXIMIZE);

    if (time > 0)
        model.set(GRB_DoubleParam_TimeLimit, time);

    model.optimize();
}

int main()
{
    vector<int> testCubeIndex{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 34, 36, 38, 40, 42, 45, 47, 49, 51,
53, 55, 57, 60, 62, 64, 66, 68, 70, 72, 77, 75, 79 };

    bitset<80> testCube;
    for (int i = 0; i < 80; i++)
        if ( count(testCubeIndex.begin(), testCubeIndex.end(), i))
            testCube[i] = 1;
        else
            testCube[i] = 0;

    bitset<288> testLastState("001111100000011000000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000100000000000000000000000000001000000000000010000000000000000000001011000001100001101000000011011000000000011011000001000011110000100011100000000000");
    
    cout << "set mipFocus to 1:" << endl;
    MidSolutionCounter(199, testCube, testLastState, 120, 2,1);

    cout << "****************************************************" << endl;

    cout << "set mipFocus to 3:" << endl;
    MidSolutionCounter(199, testCube, testLastState, 120, 2, 3);

}
