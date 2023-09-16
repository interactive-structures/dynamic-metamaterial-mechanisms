#include<vector>

#pragma once

using std::vector;

class PRand {
    private:
        int acc;
        int N;
        vector<int> store;
    public:
        PRand(int N);
        int next();
};