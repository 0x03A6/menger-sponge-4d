#ifndef RAY_HPP
#define RAY_HPP

#include <algorithm>

#include "vector.hpp"
#include "surface.hpp"
#include "space_converter.hpp"

struct Ray {
    V4d origin;
    V4d direction;
    Ray() {}
    Ray(const V4d _origin, const V4d _direction)
        :origin(_origin), direction(_direction) {}
    inline V4d at(double len) {
        return origin + len * direction;
    }
    //直线与面相交，交点 = origin + direction * 返回值
    double intersect(Surface *s) {
        //std::cout << (origin.x * s->a + origin.y * s->b + origin.z * s->c + s->d) << std::endl;
        //double t = direction.x * s->a + direction.y * s->b + direction.z * s->c;
        double t = dot(direction, *s->normal());
        if (t == 0)
            return 0;
        return -(dot(origin, *s->normal()) + s->d) / t;
    }
    double intersectValueX(double x) {
        return (x - origin.x) / direction.x;
    }
    double intersectValueY(double y) {
        return (y - origin.y) / direction.y;
    }
    double intersectValueZ(double z) {
        return (z - origin.z) / direction.z;
    }
    V4d intersectPointX(double x, double l) {
        return { x, origin.y + l * direction.y, origin.z + l * direction.z, origin.w + l * direction.w };
    }
    V4d intersectPointY(double y, double l) {
        return { origin.x + l * direction.x, y, origin.z + l * direction.z, origin.w + l * direction.w };
    }
    V4d intersectPointZ(double z, double l) {
        return { origin.x + l * direction.x, origin.y + l * direction.y, z, origin.w + l * direction.w };
    }
    V4d intersectPointW(double w, double l) {
        return { origin.x + l * direction.x, origin.y + l * direction.y, origin.z + l * direction.z, w };
    }
    //参数：顶点位置、边长，确定了一个正方形。cvt: converter
    template<typename Cvt>
    std::pair<double, V4d> intersectSquare(Cvt p, double a) {
        const double l = (p.X() - ((Cvt *)&origin)->X()) / ((Cvt *)&direction)->X();  // 交点在光线对应的前方 l 处
        Cvt intersection(p.X(), ((Cvt *)&origin)->Y() + l * ((Cvt *)&direction)->Y(), ((Cvt *)&origin)->Z() + l * ((Cvt *)&direction)->Z(), ((Cvt *)&origin)->W() + l * ((Cvt *)&direction)->W());
        const double _y = intersection.Y() - p.Y(), _z = intersection.Z() - p.Z(), _w = intersection.W() - p.W();
        if (_y > 0 && _y < a && _z > 0 && _z < a && _w > 0 && _w < a && l != 0) // l != 0 在 judgeHit 里面判过了，可能可以删掉
            return std::make_pair(l, intersection);
        else
            return std::make_pair(__builtin_inff(), intersection);
    }
    V4d intersectPoint(Surface *s) {
        return at(intersect(s));
    }
};

// BUG?
V4d reflect(V4d in, V4d normal) {
	const V4d projection = normal * (dot(in, normal) / dot(normal, normal));	// in 在 normal 方向的投影向量
	const V4d difference = in - projection;
	return projection - difference;
}

#endif