#include <iostream>
#include <vector>

namespace montecarlo
{
    struct Node
    {
        Node() : pos({0,0}), vis(0), val(0), utc(0), 
                parentND(nullptr), candiNDs({}), childNDs({}) {}
        Node(std::vector<int> pos, int vis, int val, float utc, 
            struct Node* paND, 
            std::vector<struct Node*> caNDs, 
            std::vector<struct Node*> chNDs) 
            : pos(pos), vis(vis), val(val), utc(utc), 
            parentND(paND), candiNDs(caNDs), childNDs(chNDs) {}
        std::vector<int> pos;
        int vis;
        int val;
        float utc;
        struct Node* parentND;
        std::vector<struct Node*> candiNDs;
        std::vector<struct Node*> childNDs;
    };

    class standard
    {
    public:
        void main();
        void selection();
        void expansion();
        void simulation();
        void backprop();
    };

}
    
