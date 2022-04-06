#include "../include/mcts.h"
#include "../include/pcutils.h"

namespace montecarlo
{
    void standard::main(std::vector<pointcloud::Point3D*> legs, std::vector<pointcloud::Point3D*> pts)
    {
        std::vector<float> goal = {1.0,1.0,1.0, 1.5,1.5,1.5};
        Node *rnd = get_rootND(legs);
        Node *snd = selection(rnd, pts);
        Node *end = expansion(snd);
        bool rslt = simulation(end, pts, goal);
        std::cout <<rslt << std::endl;
        memoryDelete(rnd);
    }
    Node *standard::selection(Node *nd, std::vector<pointcloud::Point3D*> pts)
    {
        if(nd->candiNDs.size() == 0 && nd->childNDs.size() == 0)
        {
            get_candiND(nd, pts);
        }
        Node* snd = new Node();
        if(nd->candiNDs.size() == 0 && nd->childNDs.size() != 0)
        {
            int max = -100;
            Node* temp_nd = new Node();
            for(int i=0; i<nd->childNDs.size(); i++)
            {
                if(nd->childNDs[i]->utc > max)
                {
                    temp_nd = nd->childNDs[i];
                    max = nd->childNDs[i]->utc;
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
    Node *standard::expansion(Node *nd)
    {
        Node *end = new Node();
        if(nd->candiNDs.size() != 0)
        {
            end = nd->candiNDs[rand()%(nd->candiNDs.size())];
            nd->childNDs.push_back(end);
            for(int i=0;i<nd->candiNDs.size();i++)
            {
                if(end==nd->candiNDs[i])
                {
                    nd->candiNDs.erase(nd->candiNDs.begin()+i);
                    break;
                }
            }
            return end;
        }
        else
        {
            return nd;
        }
    }
    bool standard::simulation(Node *nd, std::vector<pointcloud::Point3D*> pts, std::vector<float> goal)
    {
        pointcloud::Point3D* standLeg = nd->pos;
        pointcloud::Point3D* swingLeg = nd->parentND->pos;
        while(1)
        {
            pointcloud::Point3D robotCenter = get_robotcenter(standLeg, swingLeg);
            if(check_goal(robotCenter, goal))
            {
                return 1;
            }
            else:
            {

            }
        }
    }
    void standard::backprop()
    {
        std::cout << "backprop" << std::endl;
    }

    /////////////////////////////////////////////
    //////////////////// Utils //////////////////
    /////////////////////////////////////////////
    bool standard::check_goal(pointcloud::Point3D center, std::vector<float> goal)
    {  
        if(goal[0]<center.GetX() && center.GetX()<goal[3] &&
           goal[1]<center.GetY() && center.GetY()<goal[4] &&
           goal[2]<center.GetZ() && center.GetZ()<goal[5]) {return 1;}
        else {return 0;}
    }
    void standard::printNDpos(Node *nd)
    {
        std::cout << "x : " << nd->pos->GetX() << std::endl;
        std::cout << "Y : " << nd->pos->GetY() << std::endl;
        std::cout << "z : " << nd->pos->GetZ() << std::endl;
    }
    Node *standard::get_rootND(std::vector<pointcloud::Point3D*> spts)
    {
        Node* nd = new Node();
        nd->pos = spts[0];
        Node* temp = new Node(spts[1],1,0,0,NULL,{},{});
        nd->childNDs.push_back(temp);
        return nd;
    }
    pointcloud::Point3D standard::get_robotcenter(pointcloud::Point3D *p1, pointcloud::Point3D *p2)
    {
        pointcloud::Point3D temp((p1->GetX()+p2->GetX())/2,
                                 (p1->GetY()+p2->GetY())/2,
                                 (p1->GetZ()+p2->GetZ())/2);
        return temp;
    }
    float standard::get_dist(pointcloud::Point3D *p1, pointcloud::Point3D *p2)
    {
        return sqrt(pow(p1->GetX() - p2->GetX(), 2)
                  + pow(p1->GetY() - p2->GetY(), 2)
                  + pow(p1->GetZ() - p2->GetZ(), 2));
    }

    void standard::get_candiND(Node *nd, std::vector<pointcloud::Point3D*> pts)
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
    void standard::memoryDelete(Node *nd)
    {
        for(int i = 0; i< nd->candiNDs.size(); i++)
        {
            memoryDelete(nd->candiNDs[i]);
        }
        for(int i = 0; i< nd->childNDs.size(); i++)
        {
            memoryDelete(nd->childNDs[i]);
        }
        delete nd;
    }
}