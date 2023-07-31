#include "PRand.hpp"
#include <cstdlib>

using std::rand;

PRand::PRand(int N) : N{N} {
    acc = 0;
    store.resize(N);
}

int PRand::next() {
    int i = rand() % (N - acc) + acc;
    int n = store[i] == 0 ? i + 1 : store[i];
    store[i] = store[acc] == 0 ? acc + 1 : store[acc];
    acc++;
    return n - 1;
}