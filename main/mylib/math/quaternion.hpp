#ifndef MYLIB_QUATERNION_HPP
#define MYLIB_QUATERNION_HPP

#include "vector3.hpp"
#include <cmath>


namespace mylib
{
    template<typename T>
    class Quaternion
    {
    public:
        T w, x, y, z;
        Quaternion() : w(1), x(0), y(0), z(0) {}
        Quaternion(T w, T x, T y, T z) : w(w), x(x), y(y), z(z) {}
        Quaternion(const Vector3<T>& v) : w(0), x(v.x), y(v.y), z(v.z) {}
        Quaternion(const Vector3<T>& v, T theta)
        {
            T s = sin(theta / 2), c = cos(theta / 2);
            w = c;
            x = s * v.x;
            y = s * v.y;
            z = s * v.z;
        }
        /*
        - assume "from" and "to" are already unit vector
        - assume dot(from, to) is not to close to 1.0
        */
        Quaternion(const Vector3<T>& from, const Vector3<T>& to, T t)
        {
            mylib::Vector3<T> v = from.cross(to).unit();
            T theta = from.angle(to) * t;

            T s = sin(theta / 2), c = cos(theta / 2);
            w = c;
            x = s * v.x;
            y = s * v.y;
            z = s * v.z;
        }

        static Vector3<T> to_vector(const Quaternion<T>& q)
        {
            return Vector3<T>(q.x, q.y, q.z);
        }
        Vector3<T> to_vector(void) const
        {
            return to_vector(*this);
        }

        static T dot(const Quaternion<T>& a, const Quaternion<T>& b)
        {
            return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        }
        T dot(const Quaternion<T>& b) const
        {
            return dot(*this, b);
        }

        static Quaternion<T> conj(const Quaternion<T>& a)
        {
            return Quaternion<T>(a.w, -a.x, -a.y, -a.z);
        }
        Quaternion<T> conj(void) const
        {
            return conj(*this);
        }

        static Quaternion<T> add_inv(const Quaternion<T>& a)
        {
            return Quaternion<T>(-a.w, -a.x, -a.y, -a.z);
        }
        Quaternion<T> add_inv(void) const
        {
            return add_inv(*this);
        }

        static Quaternion<T> inv(const Quaternion<T>& a)
        {
            T s = dot(a, a);
            return Quaternion<T>(a.w / s, -a.x / s, -a.y / s, -a.z / s);
        }
        Quaternion<T> inv(void) const
        {
            return inv(*this);
        }

        static T norm(const Quaternion<T>& a)
        {
            return sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
        }
        T norm(void) const
        {
            return norm(*this);
        }

        static Quaternion<T> unit(const Quaternion<T>& a)
        {
            T n = norm(a);
            return Quaternion<T>(a.w / n, a.x / n, a.y / n, a.z / n);
        }
        Quaternion<T> unit(void) const
        {
            return unit(*this);
        }

        static Quaternion<T> shortest(const Quaternion<T>& a)
        {
            return a.w >= 0 ? a : add_inv(a);
        }
        Quaternion<T> shortest(void) const
        {
            return shortest(*this);
        }

        static Quaternion<T> quat_mul(const Quaternion<T>& a, const Quaternion<T>& b)
        {
            return Quaternion<T>(a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                                 a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                                 a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                                 a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
        }
        Quaternion<T> quat_mul(const Quaternion<T>& b) const
        {
            return quat_mul(*this, b);
        }

        static Vector3<T> rotate(const Quaternion<T>& q, const Vector3<T>& v)
        {
            Quaternion<T> q_conj = conj(q);
            Quaternion<T> v_quat(v);
            Quaternion<T> result = quat_mul(quat_mul(q, v_quat), q_conj);
            return result.to_vector();
        }
        Vector3<T> rotate(const Vector3<T>& v) const
        {
            return rotate(*this, v);
        }
    };
}

#endif
