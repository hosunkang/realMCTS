#include "../include/mcts.h"

namespace montecarlo
{
    void standard::main()
    {
        selection();
        expansion();
        simulation();
        backprop();
    }
    void standard::selection()
    {
        std::cout << "Selection" << std::endl;
    }
    void standard::expansion()
    {
        std::cout << "expansion" << std::endl;
    }
    void standard::simulation()
    {
        std::cout << "simulation" << std::endl;
    }
    void standard::backprop()
    {
        std::cout << "backprop" << std::endl;
    }
}