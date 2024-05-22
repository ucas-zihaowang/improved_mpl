
#include <iostream>
using namespace std;
#include "common.h"

int main( int argc, char **argv )
{
    std::cout << "Simple Test is Running" << std::endl;

    Vec2f start(2.5,-3.5);

    Vec2f goal(37, 2.5);

    std::cout << (start - goal).template lpNorm<Eigen::Infinity>() << std::endl;


    return 0;
}

