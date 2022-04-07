#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "../include/mcts.h"

int main(int argc, char**argv)
{
    pointcloud::pcdFile fi;
    std::string inputPath = "/home/hs/catkin_ws/src/real_mcts/data/without_sampling_3.pcd";
    std::vector<pointcloud::Point3D*> inputPoints = fi.ReadPCDToVector(inputPath);
    std::vector<pointcloud::Point3D*> legs;
    std::vector<pointcloud::Point3D> leftLegs;
    std::vector<pointcloud::Point3D> rightLegs;
    pointcloud::Point3D* leg1 = new pointcloud::Point3D(-0.25,0.5,0.7);
    pointcloud::Point3D* leg2 = new pointcloud::Point3D(0.25,0.5,0.7);
    legs.push_back(leg1);
    legs.push_back(leg2);
    leftLegs.push_back(*leg1);
    rightLegs.push_back(*leg2);
    //x = right, y=down, z=forward

    std::vector<float> goal = {-0.1,2.0, 0.1,2.2};
    montecarlo::standard st;
    clock_t start;
    clock_t end;

    for(int i=0; i<4;i++)
    {
        start = clock();
        pointcloud::Point3D pt = st.main(legs, inputPoints, goal);
        end = clock();
        std::cout << ((double)(end - start)) / (long)CLOCKS_PER_SEC << " sec" << std::endl;
        if(i%2 == 0)
        {
            rightLegs.push_back(*legs[0]);
        }
        else
        {
            leftLegs.push_back(*legs[0]);
        }
        std::cout << i+1 <<"th MCTS complete" << std::endl;
    }

    ////////////////////////////
    ///visuallization on Rviz///
    ////////////////////////////
    ros::init(argc, argv, "main");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("visual_marker", 10);

    ros::Rate r(30);

    while(ros::ok())
    {
        visualization_msgs::Marker points, leftLegPTs, rightLegPTs, goalcube;
        points.header.frame_id = leftLegPTs.header.frame_id = rightLegPTs.header.frame_id = "/map";
        points.header.stamp = leftLegPTs.header.stamp = rightLegPTs.header.stamp = ros::Time::now();
        points.ns = leftLegPTs.ns = rightLegPTs.ns = "points_and_lines";
        points.action = leftLegPTs.action = rightLegPTs.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = leftLegPTs.pose.orientation.w = rightLegPTs.pose.orientation.w = 1.0;

        goalcube.header.frame_id = "/map";
        goalcube.header.stamp = ros::Time::now();
        goalcube.ns = "points_and_lines";
        goalcube.action = visualization_msgs::Marker::ADD;
        goalcube.pose.orientation.w = 1.0;

        points.id = 0;
        leftLegPTs.id = 1;
        rightLegPTs.id = 2;
        goalcube.id = 3;

        points.type = visualization_msgs::Marker::POINTS;
        leftLegPTs.type = visualization_msgs::Marker::POINTS;
        rightLegPTs.type = visualization_msgs::Marker::POINTS;
        goalcube.type = visualization_msgs::Marker::CUBE;

        points.scale.x = 0.02;
        points.scale.y = 0.02;

        points.color.g = 1.0f;
        points.color.a = 1.0;
        for(int i=0; i<inputPoints.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = inputPoints[i]->GetX();
            p.y = inputPoints[i]->GetY();
            p.z = inputPoints[i]->GetZ();

            points.points.push_back(p);
        }

        leftLegPTs.scale.x = 0.05;
        leftLegPTs.scale.y = 0.05;

        leftLegPTs.color.b = 1.0;
        leftLegPTs.color.a = 1.0;
        
        for(int i=0; i<leftLegs.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = leftLegs[i].GetX();
            p.y = leftLegs[i].GetY();
            p.z = leftLegs[i].GetZ();
            leftLegPTs.points.push_back(p);
        }

        rightLegPTs.scale.x = 0.05;
        rightLegPTs.scale.y = 0.5;

        rightLegPTs.color.r = 1.0;
        rightLegPTs.color.a = 1.0;

        for(int i=0; i<rightLegs.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = rightLegs[i].GetX();
            p.y = rightLegs[i].GetY();
            p.z = rightLegs[i].GetZ();
            rightLegPTs.points.push_back(p);
        }

        goalcube.scale.x = 0.2;
        goalcube.scale.y = 0.2;
        goalcube.scale.z = 0.2;
        goalcube.color.r = 1.0;
        goalcube.color.a = 0.5;
        goalcube.pose.position.x = (goal[0]+goal[2])/2;
        goalcube.pose.position.y = 0.4;
        goalcube.pose.position.z = (goal[1]+goal[3])/2;

        pub.publish(points);
        pub.publish(goalcube);
        pub.publish(leftLegPTs);
        pub.publish(rightLegPTs);
        r.sleep();
    }

    delete leg1;
    delete leg2;
}