#pragma once

#ifndef SPACE_CONVERTER_HPP
#define SPACE_CONVERTER_HPP

#include "vector.hpp"

// 为了处理方便定义了用于坐标轴交换的三个类。
// 有点抽象。
template<typename T>
struct XYZW : public Vector4D<T> {
    T &X() { return Vector4D<T>::x; }
    T &Y() { return Vector4D<T>::y; }
    T &Z() { return Vector4D<T>::z; }
    T &W() { return Vector4D<T>::w; }
    XYZW() {}
    XYZW(V4d a) :Vector4D<T>(a) {}   //ATTENTION
    XYZW(const T x, const T y, const T z, const T w) {
        X() = x; Y() = y; Z() = z; W() = w;
    }
};
template<typename T>
struct YZWX : public Vector4D<T> {
    T &X() { return Vector4D<T>::y; }
    T &Y() { return Vector4D<T>::z; }
    T &Z() { return Vector4D<T>::w; }
    T &W() { return Vector4D<T>::x; }
    YZWX() {}
    YZWX(V4d a) :Vector4D<T>(a) {}   //ATTENTION
    YZWX(const T x, const T y, const T z, const T w) {
        X() = x; Y() = y; Z() = z; W() = w;
    }
};
template<typename T>
struct ZWXY : public Vector4D<T> {
    T &X() { return Vector4D<T>::z; }
    T &Y() { return Vector4D<T>::w; }
    T &Z() { return Vector4D<T>::x; }
    T &W() { return Vector4D<T>::y; }
    ZWXY() {}
    ZWXY(V4d a) :Vector4D<T>(a) {}   //ATTENTION
    ZWXY(const T x, const T y, const T z, const T w) {
        X() = x; Y() = y; Z() = z; W() = w;
    }
};
template<typename T>
struct WXYZ : public Vector4D<T> {
    T &X() { return Vector4D<T>::w; }
    T &Y() { return Vector4D<T>::x; }
    T &Z() { return Vector4D<T>::y; }
    T &W() { return Vector4D<T>::z; }
    WXYZ() {}
    WXYZ(V4d a) :Vector4D<T>(a) {}   //ATTENTION
    WXYZ(const T x, const T y, const T z, const T w) {
        X() = x; Y() = y; Z() = z; W() = w;
    }
};

using dXYZW = XYZW<double>;
using dYZWX = YZWX<double>;
using dZWXY = ZWXY<double>;
using dWXYZ = WXYZ<double>;

#endif