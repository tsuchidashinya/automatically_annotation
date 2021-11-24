#include <iostream>
#include <random>
#include <iomanip>

using std::cout;
using std::endl;
using std::setprecision;

constexpr int FLOAT_MIN = -3;
constexpr int FLOAT_MAX = 3;

int main()
{
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<> distr(FLOAT_MIN, FLOAT_MAX);
    std::uniform_int_distribution<> *dirst1;
    dirst1 = new std::uniform_int_distribution<>(0, 100);

    for (int n = 0; n < 20; ++n) {
        std::cout << dirst1->operator()(eng) << std::endl;
    }

    return EXIT_SUCCESS;
}
