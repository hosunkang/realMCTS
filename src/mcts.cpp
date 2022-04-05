#include "../include/mcts.h"
#include "../include/pcutils.h"

namespace montecarlo
{
    void standard::main(std::vector<pointcloud::Point3D*> legs, std::vector<pointcloud::Point3D*> pts)
    {
        Node rnd = get_rootND(legs);
        printNDpos(rnd);
        Node seleND = selection(rnd, pts);
        Node expaND = expansion(seleND);
        std::cout << "Num. of Candidate nodes of selected Node : "<< seleND.candiNDs.size() << std::endl;


        for(int i=0; i<seleND.candiNDs.size(); i++)
        {
            delete seleND.candiNDs[i];
        }
    }
    void standard::printNDpos(Node nd)
    {
        std::cout << nd.pos->GetX() << std::endl;
        std::cout << nd.pos->GetY() << std::endl;
        std::cout << nd.pos->GetZ() << std::endl;
    }

    Node standard::get_rootND(std::vector<pointcloud::Point3D*> spts)
    {
        Node nd;
        nd.pos = spts[0];
        Node* temp = new Node(spts[1],1,0,0,NULL,{},{}); //Use dynamic allocate
        nd.childNDs.push_back(temp);
        return nd;
    }
    float standard::get_dist(pointcloud::Point3D* p1, pointcloud::Point3D* p2)
    {
        return sqrt(pow(p1->GetX() - p2->GetX(), 2)
                  + pow(p1->GetY() - p2->GetY(), 2)
                  + pow(p1->GetZ() - p2->GetZ(), 2));
    }

    void standard::get_candiND(Node* nd, std::vector<pointcloud::Point3D*> pts)
    {
        for(int i=0; i<pts.size(); i++)
        {
            float width = get_dist(nd->pos, pts[i]);
            if(width < 1)
            {   
                Node* temp = new Node(pts[i],nd); 
                nd->candiNDs.push_back(temp);
            }
        }
    }
    Node standard::selection(Node nd, std::vector<pointcloud::Point3D*> pts)
    {
        if(nd.candiNDs.size() == 0 && nd.childNDs.size() == 0)
        {
            get_candiND(&nd, pts);
        }

        Node snd;
        if(nd.candiNDs.size() == 0 && nd.childNDs.size() != 0)
        {
            int max = -100;
            Node temp_nd;
            for(int i=0; i<nd.childNDs.size(); i++)
            {
                if(nd.childNDs[i]->utc > max)
                {
                    temp_nd = *nd.childNDs[i];
                    max = nd.childNDs[i]->utc;
                }
            }
            snd = standard::selection(temp_nd, pts);
        }
        else
        {
            snd = nd;
        }
        return snd;
    }
    Node standard::expansion(Node nd)
    {
        std::cout << "expansion" << std::endl;
    }
    bool standard::simulation()
    {
        std::cout << "simulation" << std::endl;
    }
    void standard::backprop()
    {
        std::cout << "backprop" << std::endl;
    }
}