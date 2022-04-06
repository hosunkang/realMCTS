#pragma once
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace pointcloud
{
    class Point2D
    {
    public:
        Point2D() : mX(0), mZ(0) {}
        Point2D(float x, float z) : mX(x), mZ(z) {}

        float GetX() const { return mX; }
        float GetZ() const { return mZ; }

        void SetX(float x) { this->mX = x; }
        void SetZ(float z) { this->mZ = z; }

    private:
        float mX, mZ;
    };

    class Point3D
    {
    public:
        Point3D() : mX(0), mY(0), mZ(0) {}
        Point3D(float x, float y, float z) : mX(x), mY(y), mZ(z) {}

        float GetX() const { return mX; }
        float GetY() const { return mY; }
        float GetZ() const { return mZ; }

        void SetX(float x) { mX = x; }
        void SetY(float y) { mY = y; }
        void SetZ(float z) { mZ = z; }
        void SetXYZ(float x, float y, float z) { this->mX = x; this->mY = y; this->mZ = z; }

    private:
        float mX, mY, mZ;
    };

    class pcdFile
    {
    public:
        std::vector<Point3D*> ReadPCDToVector(std::string inputPath);      // modified

    private:
        std::vector<Point3D*> mPoints;

    };

}
