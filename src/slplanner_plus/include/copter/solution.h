
#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "primitive.h"

template <int Dim>
struct Solution
{
    vec_E<CopterState<Dim>> states;
    std::vector<int> actions;
    std::vector<double> costs;
    vec_E<Primitive<Dim>> prs;

    double totalcost{0};
    double timecost{0};
    double timestampcost{0};
    double controlcost{0};

    void printSolutionInfo()
    {
        std::cout << ANSI_COLOR_GREEN << "********************Solution Result********************" << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN << "Total Cost: " << totalcost  << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN << "Time Cost: " << timecost  << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN << "Timestamp Cost: " << timestampcost  << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN << "Control Cost: " << controlcost  << ANSI_COLOR_RESET << std::endl;



        // for ( int i = 0; i < states_num; i++ )
        // {
        //     std::string info = "State " + std::to_string(i);
        //     states[i].printCopterStateInfo(info);
        // }

        std::cout << ANSI_COLOR_GREEN << "********************End********************" << ANSI_COLOR_RESET << std::endl;
    }
};

typedef Solution<2> Solution2d;
typedef Solution<3> Solution3d;

#endif
