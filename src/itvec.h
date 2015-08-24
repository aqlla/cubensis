#ifndef __it_VEC_h__
#define __it_VEC_h__

#include "Arduino.h"
#include "math.h"
#include "util.h"

#if defined(Arduino_h)
#define VEC_PRINT(x)    Serial.print(x)
#define VEC_PRINTLN(x)  Serial.print(x); Serial.print("\n")
#else
    #include <iostream>
    #define VEC_PRINT(x)    std::cout << x
    #define VEC_PRINTLN(x)  std::cout << x << std::endl;
#endif

#define RAD2DEG 57.29577

namespace it
{
    template <typename T>
    struct vec3
    {
        T arr[3];
        T& x = arr[0];
        T& y = arr[1];
        T& z = arr[2];


        vec3(vec3<T>& vec3)
                : arr{vec3.x, vec3.y, vec3.z} {};

        vec3(T x = 0, T y = 0, T z = 0)
                : arr{x, y, z} {};

        vec3(T* arr)
                : arr{arr[0], arr[1], arr[2]} {};

        virtual ~vec3() {};

        void set(T x, T y, T z) {
            this->x = x;
            this->y = y;
            this->z = z;
        };

        // Math functions
        cfloat magnitude() {
            return sqrt(x * x + y * y + z * z);
        };

        vec3<cfloat> normalized() {
            return vec3<cfloat>(x, y, z).normalize();
        };

        vec3<T>& normalize() {
            cfloat magnitude = this->magnitude();
            x /= magnitude;
            y /= magnitude;
            z /= magnitude;
            return * this;
        };

        T dot(const vec3<T>& rhs) {
            return x * rhs.x + y * rhs.y + z * rhs.z;
        };

        vec3<T> cross(const vec3<T>& rhs) {
            return vec3<T>(
                    (y * rhs.z) - (z * rhs.y),
                    (z * rhs.x) - (x * rhs.z),
                    (x * rhs.y) - (y * rhs.x)
            );
        };

        // Operator overloading
        virtual void operator =(const vec3<T>& rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        };

        vec3<T> operator +(const vec3<T>& rhs) {
            return vec3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
        };

        vec3<T> operator +(const T rhs) {
            return vec3<T>(x + rhs, y + rhs, z + rhs);
        };

        vec3<T> operator -(const vec3<T>& rhs) {
            return vec3<T>(x - rhs.x, y - rhs.y, z - rhs.z);
        };

        vec3<T> operator -(const T rhs) {
            return vec3<T>(x - rhs, y - rhs, z - rhs);
        };

        vec3<T> operator /(const vec3<T>& rhs) {
            return vec3<T>(x / rhs.x, y / rhs.y, z / rhs.z);
        };

        vec3<cfloat> operator /(const T rhs) {
            return vec3<cfloat>(x / (cfloat) rhs, y / (cfloat) rhs, z / (cfloat) rhs);
        };

        vec3<T> operator *(const vec3<T>& rhs) {
            return vec3<T>(x * rhs.x, y * rhs.y, z * rhs.z);
        };

        vec3<T> operator *(const T rhs) {
            return vec3<T>(x * rhs, y * rhs, z * rhs);
        };


        vec3<T>& operator +=(const vec3<T>& rhs) {
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;
            return * this;
        };

        vec3<T>& operator +=(const T rhs) {
            x += rhs;
            y += rhs;
            z += rhs;
            return * this;
        };

        vec3<T>& operator -=(const vec3<T>& rhs) {
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;
            return * this;
        };

        vec3<T>& operator -=(const T rhs) {
            x -= rhs;
            y -= rhs;
            z -= rhs;
            return * this;
        };

        vec3<T>& operator /=(const vec3<T>& rhs) {
            x /= rhs.x;
            y /= rhs.y;
            z /= rhs.z;
            return * this;
        };

        vec3<T>& operator /=(const T rhs) {
            x /= rhs;
            y /= rhs;
            z /= rhs;
            return * this;
        };

        vec3<T>& operator *=(const vec3<T>& rhs) {
            x *= rhs.x;
            y *= rhs.y;
            z *= rhs.z;
            return * this;
        };

        vec3<T>& operator *=(const T rhs) {
            x *= rhs;
            y *= rhs;
            z *= rhs;
            return * this;
        };

        operator T*() {
            return & arr[0];
        }

        T& operator [](int index) {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                default:
                    return x;
            }
        };

        const T& operator [](int index) const {
            switch (index) {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                default:
                    return x;
            }
        };

        void print() {
            VEC_PRINT("x: ");
            VEC_PRINT(x);
            VEC_PRINT(", y: ");
            VEC_PRINT(y);
            VEC_PRINT(", z: ");
            VEC_PRINT(z);
            VEC_PRINTLN("\n");
        };
    };

    template <typename T>
    vec3<T> operator -(const T lhs, const vec3<T>& rhs) {
        return vec3<T>(lhs - rhs.x, lhs - rhs.y, lhs - rhs.z);
    };

    template <typename T>
    vec3<T> operator -(const vec3<T>& lhs, const T rhs) {
        return vec3<T>(lhs.x - rhs, lhs.y - rhs, lhs.z - rhs);
    };


    struct AccelerationVec : public vec3<cfloat>
    {
        AccelerationVec(cfloat x = 0, cfloat y = 0, cfloat z = 0)
                : vec3<cfloat>(x, y, z) {}

        double roll() {
            return atan2(y, sqrt(x * x + z * z)) * RAD2DEG;
        };

        double pitch() {
            return atan2(x, sqrt(y * y + z * z)) * RAD2DEG;
        };

        // Operator overloading
        template <typename T>
        void operator =(const vec3<T>& rhs) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        };
    };
};


#endif
