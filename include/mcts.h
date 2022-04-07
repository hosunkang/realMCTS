#include <iostream>
#include <vector>
#include <cmath>
#include <random>
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
        
        pointcloud::Point3D *pos;
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
        pointcloud::Point3D main(std::vector<pointcloud::Point3D*>, std::vector<pointcloud::Point3D*>, std::vector<float>);

        // MCTS main process
        Node *get_rootND(std::vector<pointcloud::Point3D*>);
        Node *selection(Node*, std::vector<pointcloud::Point3D*>);
        Node *expansion(Node*);
        bool simulation(Node*, std::vector<pointcloud::Point3D*>, std::vector<float>);
        void backprop(bool, Node*);
        pointcloud::Point3D finalSelect(Node*);

        // MCTS utils
        void printNDpos(Node*);
        void get_candiND(Node*, std::vector<pointcloud::Point3D*>);
        float get_dist(pointcloud::Point3D*, pointcloud::Point3D*);
        pointcloud::Point3D get_robotcenter(pointcloud::Point3D*, pointcloud::Point3D*); 
        bool check_goal(pointcloud::Point3D, std::vector<float>);
        std::vector<pointcloud::Point3D*> get_simulND(pointcloud::Point3D*,pointcloud::Point3D*, std::vector<pointcloud::Point3D*>);
        float utcFunc(Node*);
        void memoryDelete(Node*);
    private:
        bool stepleg;
    };

}
    
