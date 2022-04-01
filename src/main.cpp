#include "../include/pcutils.h"
#include "../include/mcts.h"

int main()
{
    pointcloud::Node nd;
    std::string inputPath = "/home/hs/catkin_ws/src/real_mcts/data/with_sampling_3.pcd";
    std::vector<pointcloud::Point3D*> inputPoints = nd.ReadPCDToVector(inputPath);

    std::cout << inputPoints.size() << std::endl;

    montecarlo::standard st;
    st.print();
}