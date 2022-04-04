#include "../include/pcutils.h"

namespace pointcloud
{
    std::vector<Point3D*> pcdFile::ReadPCDToVector(std::string inputPath)
    {
        std::ifstream fin;
        fin.open(inputPath);

        std::string line;

        if (fin.is_open())
        {
            int num = 1;
            while (!fin.eof())
            {
                getline(fin, line);
                if (num > 10)
                {
                    float x, y, z;
                    std::istringstream iss(line);
                    iss >> x >> y >> z;

                    Point3D* pointXYZ = new Point3D(x, y, z);
                    mPoints.push_back(pointXYZ);
                }
                num++;
            }
        }
        fin.close();
        return mPoints;
    }
}