#pragma once
#include <iostream>
#include <Eigen/Geometry>

namespace velodyne_cluster
{
    class Vect3D
    {
        float x_, y_, z_;

    public:
        explicit Vect3D() = default;
        explicit Vect3D(const float &setX,
              const float &setY,
              const float &setZ)
            : x_(setX), y_(setY), z_(setZ) {}
        ~Vect3D() {}

        float getX() const { return x_; }
        float getY() const { return y_; }
        float getZ() const { return z_; }

        void setX(const float &x) { x_ = x; }
        void setY(const float &y) { y_ = y; }
        void setZ(const float &z) { z_ = z; }

        Vect3D operator+(const Vect3D &vec)
        {
            Vect3D result{x_ + vec.x_, y_ + vec.y_, z_ + vec.z_};
            return result;
        }

        Vect3D operator-(const Vect3D &vec)
        {
            Vect3D result{x_ - vec.x_, y_ - vec.y_, z_ - vec.z_};
            return result;
        }
    };

    std::ostream& operator<<(std::ostream& os, const Vect3D& vec)
    {
        os << "x: " <<vec.getX() << " y: " << vec.getY() << " z: " << vec.getZ();
        return os;
    }
}
