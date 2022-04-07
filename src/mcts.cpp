#include "../include/mcts.h"

namespace montecarlo
{
    pointcloud::Point3D standard::main(std::vector<pointcloud::Point3D*> legs, std::vector<pointcloud::Point3D*> pts, std::vector<float> goal)
    {
        Node *rnd = get_rootND(legs);;
        for(int i=0; i<1000;i++)
        {
            Node *snd = selection(rnd, pts);
            Node *end = expansion(snd);
            bool rslt = simulation(end, pts, goal);
            backprop(rslt, end);
        }
        pointcloud::Point3D finalPT = finalSelect(rnd->childNDs[0]);
        legs[1]->SetXYZ(legs[0]->GetX(), legs[0]->GetY(), legs[0]->GetZ());
        legs[0]->SetXYZ(finalPT.GetX(), finalPT.GetY(), finalPT.GetZ());
        memoryDelete(rnd);
        return finalPT;
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
            float max = -10.0;
            Node* temp_nd = new Node();
            for(int i=0; i<nd->childNDs.size(); i++)
            {
                if(nd->childNDs[i]->utc > max)
                {
                    temp_nd = nd->childNDs[i];
                    max = temp_nd->utc;
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
            end = nd->candiNDs[rand()%nd->candiNDs.size()];
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
        std::vector<pointcloud::Point3D*> candiPTs;
        while(1)
        {
            pointcloud::Point3D robotCenter = get_robotcenter(standLeg, swingLeg);
            if(check_goal(robotCenter, goal))
            {
                return 1;
            }
            else
            {
                candiPTs = get_simulND(standLeg, swingLeg, pts);
                if(candiPTs.size()==0)
                {
                    return 0;
                }
                swingLeg = standLeg;
                standLeg = candiPTs[rand()%candiPTs.size()];
            }
        }
    }
    void standard::backprop(bool result, Node *nd)
    {
        nd->vis += 1;
        if(result == 1)
        {
            nd->val += 1;
        }
        if(nd->parentND != NULL)
        {
            backprop(result, nd->parentND);
            for(int i=0; i< nd->parentND->childNDs.size(); i++)
            {
                nd->parentND->childNDs[i]->utc = utcFunc(nd->parentND->childNDs[i]);
            }
        }
    }
    pointcloud::Point3D standard::finalSelect(Node* nd)
    {
        float max = -10.0;
        Node *maxND = new Node();
        int index = 0;
        for(int i=0; i<nd->childNDs.size(); i++)
        {
            float temp = float(nd->childNDs[i]->val) / float(nd->childNDs[i]->vis);
            // std::cout << i << "th: ";
            // std::cout << temp << "  ";
            // std::cout << nd->childNDs[i]->pos->GetX() << "  ";
            // std::cout << nd->childNDs[i]->pos->GetY() << "  ";
            // std::cout << nd->childNDs[i]->pos->GetZ() << std::endl;
            if(temp > max)
            { 
                maxND = nd->childNDs[i];
                max = temp;
                index = i;
            }
        }
        return *maxND->pos;
    }

    /////////////////////////////////////////////
    //////////////////// Utils //////////////////
    /////////////////////////////////////////////
    float standard::utcFunc(Node* nd)
    {
        int w = nd->val;
        int n = nd->vis;
        int t = nd->parentND->vis;
        if(n != 0)
        {
            float utc = float(w)/n+sqrt(2*log10(t)/n);
            return utc;
        }            
        else 
        {
            return 0;
        }
    }
    bool standard::check_goal(pointcloud::Point3D center, std::vector<float> goal)
    {  
        if(goal[0]<center.GetX() && center.GetX()<goal[2] &&
           goal[1]<center.GetZ() && center.GetZ()<goal[3]) {return 1;}
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
        Node* nd = new Node(spts[0],1,0,0,NULL,{},{});
        nd->pos = spts[0];
        Node* temp = new Node(spts[1],1,0,0,nd,{},{});
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
            float distance = get_dist(nd->parentND->pos, pts[i]);
            float forward = pts[i]->GetZ() - nd->parentND->pos->GetZ();
            float width = pts[i]->GetX() - nd->parentND->pos->GetX();
            float y1 = pts[i]->GetX() - nd->pos->GetX();
            float y2 = nd->parentND->pos->GetX() - nd->pos->GetX();

            if(0.4 < distance && distance< 0.6 && forward > 0 && std::abs(width) > 0.4)
            {   
                if(std::abs(y1) < std::abs(y2))
                {
                    Node* temp = new Node(pts[i],nd); 
                    nd->candiNDs.push_back(temp);
                }
            }
        }
    }
    std::vector<pointcloud::Point3D*> standard::get_simulND(pointcloud::Point3D* stand,pointcloud::Point3D* swing, std::vector<pointcloud::Point3D*> pts)
    {
        std::vector<pointcloud::Point3D*> candis;
        for(int i=0; i<pts.size(); i++)
        {
            float distance = get_dist(stand, pts[i]);
            float forward = pts[i]->GetZ() - stand->GetZ();
            float width = pts[i]->GetX() - stand->GetX();
            float y1 = pts[i]->GetX() - swing->GetX();
            float y2 = stand->GetX() - swing->GetX();

            if(0.4 < distance && distance< 0.6 && forward > 0 && std::abs(width) > 0.4)
            {
                if(std::abs(y1) < std::abs(y2))
                {
                    candis.push_back(pts[i]);
                }
            }
        }
        return candis;
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