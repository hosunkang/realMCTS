#include <iostream>
#include <vector>
#include <cmath>
#include "../include/pcutils.h"

namespace montecarlo
{
    struct Node
    {
        Node() : pos(0), vis(0), val(0), utc(0), 
                parentND(nullptr), candiNDs({}), childNDs({}) {}
        Node(pointcloud::Point3D* pos, struct Node* paND) 
            : pos(pos), vis(0), val(0), utc(0), 
            parentND(paND), candiNDs({}), childNDs({}) {}
        Node(pointcloud::Point3D* pos, int vis, int val, float utc, 
            struct Node* paND, 
            std::vector<struct Node*> caNDs, 
            std::vector<struct Node*> chNDs) 
            : pos(pos), vis(vis), val(val), utc(utc), 
            parentND(paND), candiNDs(caNDs), childNDs(chNDs) {}
        
        pointcloud::Point3D* pos;
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
        void main(std::vector<pointcloud::Point3D*> legs, std::vector<pointcloud::Point3D*> pts);

        void printNDpos(Node nd);
        Node get_rootND(std::vector<pointcloud::Point3D*> spts);
        void get_candiND(Node* nd, std::vector<pointcloud::Point3D*> pts);
        float get_dist(pointcloud::Point3D* pt1, pointcloud::Point3D* pt2);

        Node selection(Node nd, std::vector<pointcloud::Point3D*> pts);
        Node expansion(Node nd);
        bool simulation();
        void backprop();
    private:
        bool stepleg;
    };

}
    
