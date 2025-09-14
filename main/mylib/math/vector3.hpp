#ifndef MYLIB_VECTOR3_HPP
#define MYLIB_VECTOR3_HPP

#include <algorithm>
#include <cmath>


namespace mylib
{
    template<typename T>
    class Vector3
    {
    public:
        T x, y, z;

        Vector3() : x(0), y(0), z(0) {}
        Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

        static Vector3<T> add_inv(const Vector3<T>& a)
        {
            return Vector3<T>(-a.x, -a.y, -a.z);
        }
        Vector3<T> add_inv(void) const
        {
            return add_inv(*this);
        }

        static Vector3<T> add(const Vector3<T>& a, const Vector3<T>& b)
        {
            return Vector3<T>(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        Vector3<T> add(const Vector3<T>& b) const
        {
            return add(*this, b);
        }

        static Vector3<T> mul(const Vector3<T>& a, const Vector3<T>& b)
        {
            return Vector3<T>(a.x * b.x, a.y * b.y, a.z * b.z);
        }
        Vector3<T> mul(const Vector3<T>& b) const
        {
            return mul(*this, b);
        }
        static Vector3<T> mul(const Vector3<T>& a, const T& scaler)
        {
            return Vector3<T>(a.x * scaler, a.y * scaler, a.z * scaler);
        }
        Vector3<T> mul(const T& scaler) const
        {
            return mul(*this, scaler);
        }

        static T dot(const Vector3<T>& a, const Vector3<T>& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }
        T dot(const Vector3<T>& b) const
        {
            return dot(*this, b);
        }

        static T norm(const Vector3<T>& a)
        {
            return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        }
        T norm(void) const
        {
            return norm(*this);
        }

        static Vector3<T> unit(const Vector3<T>& a)
        {
            T n = norm(a);
            return Vector3<T>(a.x / n, a.y / n, a.z / n);
        }
        Vector3<T> unit(void) const
        {
            return unit(*this);
        }

        static T angle(const Vector3<T>& a, const Vector3<T>& b)
        {
            return acos(std::clamp(dot(a, b) / (norm(a) * norm(b)), -1.0, 1.0));
        }
        T angle(const Vector3<T>& b) const
        {
            return angle(*this, b);
        }

        static Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b)
        {
            return Vector3<T>(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }
        Vector3<T> cross(const Vector3<T>& b) const
        {
            return cross(*this, b);
        }
    };
}

#endif
