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
        Point2D GetEndNodeXZ() const { return mEndNodeXZ; }

        void SetX(float x) { this->mX = x; }
        void SetY(float y) { this->mY = y; }
        void SetZ(float z) { this->mZ = z; }
        void SetXYZ(float x, float y, float z) { this->mX = x; this->mY = y; this->mZ = z; }
        void SetEndNodeXZ(Point2D endNodeXZ) { this->mEndNodeXZ = endNodeXZ; }
        void SetEndNodeXZ(float x, float z) { this->mEndNodeXZ.SetX(x); this->mEndNodeXZ.SetZ(z); }

    private:
        float mX, mY, mZ;
        Point2D mEndNodeXZ;
    };

    class pcdFile
    {
    public:
        std::vector<Point3D*> ReadPCDToVector(std::string inputPath);      // modified

    private:
        std::vector<Point3D*> mPoints;

    };

}
