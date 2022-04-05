#include "ros/ros.h"
#include "../include/pcutils.h"
#include "../include/mcts.h"


int main()
{
    pointcloud::pcdFile fi;
    std::string inputPath = "/home/hs/catkin_ws/src/real_mcts/data/with_sampling_3.pcd";
    std::vector<pointcloud::Point3D*> inputPoints = fi.ReadPCDToVector(inputPath);
    std::vector<pointcloud::Point3D*> legs;
    pointcloud::Point3D* leg1 = new pointcloud::Point3D(-0.05,0,0);
    pointcloud::Point3D* leg2 = new pointcloud::Point3D(0.05,0,0);
    legs.push_back(leg1);
    legs.push_back(leg2);
    //x = right, y=down, z=forward

    montecarlo::standard st;
    st.main(legs, inputPoints);
}