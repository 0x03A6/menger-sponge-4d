#pragma once
#ifndef SPONGE_HPP
#define SPONGE_HPP

#include <algorithm>
#include <random>

#include "vector.hpp"
#include "color.hpp"
#include "ray.hpp"
#include "stack.hpp"
#include "space_converter.hpp"

#define INF (INFINITY)

constexpr int W = 800;
constexpr int H = 600;
constexpr int SPONGE_L = 3; // 递归时每边细分为的小格数（门格海绵为 3）
constexpr double DIST_EPS = 1e-10;  // 小于这个数的算作距离的计算误差
constexpr V4d INIT_CAMERA_FRONT     (-1.01, -1.02, -1.01, 0.0);
constexpr V4d INIT_CAMERA_HEAD      (-1, 0, 1, 0);
constexpr V4d INIT_CAMERA_SOMEWHERE (0, 0, 0, 1);
constexpr V4d INIT_CAMERA_POS       (SPONGE_L * 2.01, SPONGE_L * 2.02, SPONGE_L * 2.03, DIST_EPS);
// constexpr V4d INIT_CAMERA_FRONT     (0.0, -1.01, -1.02, -1.01);
// constexpr V4d INIT_CAMERA_HEAD      (0, -1, 0, 1);
// constexpr V4d INIT_CAMERA_SOMEWHERE (1, 0, 0, 0);
// constexpr V4d INIT_CAMERA_POS       (DIST_EPS, SPONGE_L * 2.01, SPONGE_L * 2.02, SPONGE_L * 2.03);
constexpr double CAMERA_AXIS = 3;
constexpr double INIT_MOVE_VELOCITY = 0.9;
constexpr double ROTATE_VELOCITY = 0.3;
constexpr int RELATIVE_FRACTAL_ITER = 8;    // 分形从现在相机在的递归层数往里渲染几层
constexpr int SCALE = 300;
constexpr double REFLECT_PROBABILITY = 0.9;
constexpr Color SKY_COLOR(0.0, 0.0, 0.0);
constexpr double SPONGE_ROUGHNESS = 1.0;    // 海绵的粗糙程度（和正规定义不太一样，详见 deflectRay）
constexpr int N = 100005;   // 一个很大的数，用于最大帧数与最大海绵递归层数 待优化

// 这些是另一个视角的相机设置
// constexpr V4d INIT_CAMERA_FRONT (-1, 0, -0);
// constexpr V4d INIT_CAMERA_HEAD  (0, 0, 1);
// constexpr V4d INIT_CAMERA_POS   (11.0 / 18.0 - 0.00001, 1.0 / 6.0 - 0.0001, 1.0 / 6.0 - 0.000001);

// random number generator
std::mt19937 rng;

double move_velocity = INIT_MOVE_VELOCITY;

bool sgn(double x) {
    if (x > 0)  return true;
    return false;
}

//描述了门格海绵长啥样，pos 代表的单元是实心还是空心，pos 从 0 开始。
bool inSponge(Vector4D<int> pos) {
    //return false;   //DEBUG
    if (pos.w != 1) {
        if (pos.x != 1)
            return (pos.y != 1 || pos.z != 1);
        //else
            return (pos.y != 1 && pos.z != 1);
    }
    return pos.x != 1 && pos.y != 1 && pos.z != 1;
}

/// 改了一下代码，把 a1 换成了 0，把 a2 换成了 3，所以里面一些东西可能看起来很屎，但是我相信会被优化掉
/// @brief 判断光线会（先）撞到两个平行的面中的哪个面
/// @param a1 第一个面的法向量方向上的坐标的相对值（对于平行于坐标轴的面，“相对值”直接用法向量对应坐标轴的坐标数值即可）
/// @param a2 第二个面……的相对值（同上）
/// @param origin 光线起点在法向量方向上的相对值
/// @param direction 光线方向在法向量方向的相对值
/// @return 如果是 `false`，则会（先）撞到 `a1` 对应的面，`true` 则 `a2`。如果都撞不上就随便返回一个
bool judgeHit(double origin, double direction) {
    const bool close_to_0 = (std::abs(origin - 0) < DIST_EPS);
    const bool close_to_1 = (std::abs(origin - SPONGE_L) < DIST_EPS);
    if (close_to_0 || close_to_1)
        return close_to_0;    // 如果 origin 在其中一个面上，忽略 origin 在的面。
    if ((origin > 0) == (origin > SPONGE_L)) //如果 a1 与 a2 在 origin 同侧
        return (direction < 0); //直接找 direction 方向较近的一个
    return (direction > 0);    //在异侧 找 direction 方向面在 origin 前方的一个
}

V4d enterBlock(V4d v, Vector4D<int> pos) {
    return  {
        (v.x - pos.x) * SPONGE_L,
        (v.y - pos.y) * SPONGE_L,
        (v.z - pos.z) * SPONGE_L,
        (v.w - pos.w) * SPONGE_L
    };
}

V4d exitBlock(V4d v, Vector4D<int> pos) {
    return {
        v.x / SPONGE_L + pos.x,
        v.y / SPONGE_L + pos.y,
        v.z / SPONGE_L + pos.z,
        v.w / SPONGE_L + pos.w
    };
}

Stack<Vector4D<int>, N> init_blocks;

V4d enterInitBlock(V4d v) {
    for (Vector4D<int> *i = init_blocks.arr + 1; i <= init_blocks.cursor; i++)
        v = enterBlock(v, *i);
    return v;
}

void calcInitBlock(V4d cam_pos) {
    move_velocity = INIT_MOVE_VELOCITY;
    init_blocks.clear();
    init_blocks.arr[0] = { 0, 0, 0, 0 };
    if (cam_pos.x <= 0 || cam_pos.x >= SPONGE_L || cam_pos.y <= 0 || cam_pos.y >= SPONGE_L || cam_pos.z <= 0 || cam_pos.z >= SPONGE_L || cam_pos.w <= 0 || cam_pos.w >= SPONGE_L) {
        return;
    }
    Vector4D<int> pos;
    while (init_blocks.size() < N) {
        pos = { cam_pos.x, cam_pos.y, cam_pos.z, cam_pos.w };
        init_blocks.push(pos);
        cam_pos = enterBlock(cam_pos, pos);
        move_velocity /= SPONGE_L;
        if (!inSponge(pos))
            return;
    }
    if (init_blocks.size() == N) {
        std::cout << "init block overflowed.\n";
        getchar();
        exit(0);
    }
}

Ray res_ray;    // 待优化
V4d res_normal;
Stack<Vector4D<int>, N> blocks;
Stack<Vector4D<int>, N> temp_blocks;  //进行 calc 计算穿透一个方块进入下一个方块 之前 栈的状态。方便光追。

/// @brief 光线入射到面上之后，处理（栈中的）方块转换
/// @tparam CvtDouble space converter of double varibles
/// @tparam CvtInt space converter of int varibles
/// @param p 线面交点
/// @note 这么长其实是因为相似代码复制了一遍
template<typename CvtDouble, typename CvtInt>
CvtDouble calc(CvtDouble p, const bool up) {
    
    //每次在函数内判断 first_in 会损失一点性能，以后有时间管一下。
    // if (p.x < 0 || p.x > SPONGE_L || p.y < 0 || p.y > SPONGE_L || p.z < 0 || p.z > SPONGE_L || p.w < 0 || p.w > SPONGE_L)
    //     std::cout << "bad\n";

    if ((!up) ^ blocks.empty()) {    //如果是第一次入射，那么碰到上表面是从上方射入新立方体；如果不是第一次入射，那么碰到旧立方体的下表面是从上方射入新立方体。
        if (!blocks.empty()) {
            while (!blocks.empty() && ((CvtInt *)&(blocks.top()))->X() == 0) {   //出方块 【改】
                p = exitBlock(p, blocks.top());
                blocks.pop();
            }
            if (blocks.empty())
                return p;
            ((CvtInt *)&(blocks.top()))->X()--; //【改】
            p.X() = SPONGE_L;  //【改】
            if (!inSponge(blocks.top()))
                return p; //进入空方块：返回，进行下一次传播（propagate）。
        }
        while (blocks.size() < init_blocks.size() + RELATIVE_FRACTAL_ITER) {  //进方块
            Vector4D<int> pos;
            pos = (CvtInt){ SPONGE_L - 1, (int)p.Y(), (int)p.Z(), (int)p.W() }; //【改】
            p = enterBlock(p, pos);
            blocks.push(pos);
            if (!inSponge(pos))
                break;
        }
    } else {
        if (!blocks.empty()) {
            while (!blocks.empty() && ((CvtInt *)&(blocks.top()))->X() == SPONGE_L - 1) {   //出方块 【改】
                p = exitBlock(p, blocks.top());
                blocks.pop();
            }
            if (blocks.empty())
                return p;
            ((CvtInt *)&(blocks.top()))->X()++; //【改】
            p.X() = 0;  //【改】
            if (!inSponge(blocks.top()))
                return p; //进入空方块：返回，进行下一次传播（propagate）。
        }
        while (blocks.size() < init_blocks.size() + RELATIVE_FRACTAL_ITER) {  //进方块
            Vector4D<int> pos;
            pos = (CvtInt){ 0, (int)p.Y(), (int)p.Z(), (int)p.W() }; //【改】
            p = enterBlock(p, pos);
            blocks.push(pos);
            if (!inSponge(pos))
                break;
        }
    }
    return p;
}

short hitWhichSurface(Ray ray) {
    bool nx = false, ny = false, nz = false, nw = false;    // 各个轴的正方向有没有反转
    if (ray.direction.x >= 0) {
        ray.direction.x = -ray.direction.x;
        ray.origin.x = SPONGE_L - ray.origin.x;
        nx = true;
    }
    if (ray.direction.y >= 0) {
        ray.direction.y = -ray.direction.y;
        ray.origin.y = SPONGE_L - ray.origin.y;
        ny = true;
    }
    if (ray.direction.z >= 0) {
        ray.direction.z = -ray.direction.z;
        ray.origin.z = SPONGE_L - ray.origin.z;
        nz = true;
    }
    if (ray.direction.w >= 0) {
        ray.direction.w = -ray.direction.w;
        ray.origin.w = SPONGE_L - ray.origin.w;
        nw = true;
    }
    // 需保证 ray.direction 已标准化。
    ray.origin.x /= -ray.direction.x;
    ray.origin.y /= -ray.direction.y;
    ray.origin.z /= -ray.direction.z;
    ray.origin.w /= -ray.direction.w;
    if (ray.origin.x < ray.origin.y && ray.origin.x < ray.origin.z && ray.origin.x < ray.origin.w)
        return nx;
    if (ray.origin.y < ray.origin.z && ray.origin.y < ray.origin.w)
        return 2 | ny;
    if (ray.origin.z < ray.origin.w)
        return 4 | nz;
    return 6 | nw;
}

//单次光线计算
//return: 若撞到面了就返回 true
bool propagate(Ray ray) {

    // // 待删除
    // const bool x_hit = judgeHit(ray.origin.x, ray.direction.x);
    // const bool y_hit = judgeHit(ray.origin.y, ray.direction.y);
    // const bool z_hit = judgeHit(ray.origin.z, ray.direction.z);
    // std::pair<double, V4d> x_hit_info = ray.intersectSquare<dXYZW>((dXYZW){ (double)x_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    // std::pair<double, V4d> y_hit_info = ray.intersectSquare<dYZWX>((dYZWX){ (double)y_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    // std::pair<double, V4d> z_hit_info = ray.intersectSquare<dZWXY>((dZWXY){ (double)z_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    // std::pair<double, V4d> w_hit_info = ray.intersectSquare<dWXYZ>((dWXYZ){ (double)z_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    // if (x_hit_info.first <= 0)  x_hit_info.first = INF; // 这三句在 judgeHit 里面好像判过了，可能可以删掉。
    // if (y_hit_info.first <= 0)  y_hit_info.first = INF; // 这三句在 judgeHit 里面好像判过了，可能可以删掉。
    // if (z_hit_info.first <= 0)  z_hit_info.first = INF; // 这三句在 judgeHit 里面好像判过了，可能可以删掉。
    // if (w_hit_info.first <= 0)  w_hit_info.first = INF; // 这四句在 judgeHit 里面好像判过了，可能可以删掉。
    // const double temp_at = std::min({ x_hit_info.first, y_hit_info.first, z_hit_info.first, w_hit_info.first });
    // if (temp_at == INF) {
    //     res_ray.direction = ray.direction;
    //     return false;
    // }

    // temp_blocks = blocks;   // 非光追可删
    
    // if      (x_hit_info.first == temp_at)   { ray.origin = calc<dXYZW, XYZW<int> >(x_hit_info.second, x_hit); res_normal = { 1, 0, 0, 0 }; }
    // else if (y_hit_info.first == temp_at)   { ray.origin = calc<dYZWX, YZWX<int> >(y_hit_info.second, y_hit); res_normal = { 0, 1, 0, 0 }; }
    // else if (z_hit_info.first == temp_at)   { ray.origin = calc<dZWXY, ZWXY<int> >(z_hit_info.second, z_hit); res_normal = { 0, 0, 1, 0 }; }
    // else if (w_hit_info.first == temp_at)   { ray.origin = calc<dZWXY, ZWXY<int> >(w_hit_info.second, z_hit); res_normal = { 0, 0, 1, 0 }; }

    // if (blocks.size() >= init_blocks.size() + RELATIVE_FRACTAL_ITER) {
    //     res_ray = ray;
    //     return true;
    // }

    while (true) {
        switch(hitWhichSurface(ray)) {
            case 0: ray.origin = calc<dXYZW, XYZW<int> >(ray.intersectSquare<dXYZW>((dXYZW){        0, 0, 0, 0 }, SPONGE_L).second, false); res_normal = { 1, 0, 0, 0 }; break;
            case 1: ray.origin = calc<dXYZW, XYZW<int> >(ray.intersectSquare<dXYZW>((dXYZW){ SPONGE_L, 0, 0, 0 }, SPONGE_L).second, true); res_normal = { 1, 0, 0, 0 }; break;
            case 2: ray.origin = calc<dYZWX, YZWX<int> >(ray.intersectSquare<dYZWX>((dYZWX){        0, 0, 0, 0 }, SPONGE_L).second, false); res_normal = { 0, 1, 0, 0 }; break;
            case 3: ray.origin = calc<dYZWX, YZWX<int> >(ray.intersectSquare<dYZWX>((dYZWX){ SPONGE_L, 0, 0, 0 }, SPONGE_L).second, true); res_normal = { 0, 1, 0, 0 }; break;
            case 4: ray.origin = calc<dZWXY, ZWXY<int> >(ray.intersectSquare<dZWXY>((dZWXY){        0, 0, 0, 0 }, SPONGE_L).second, false); res_normal = { 0, 0, 1, 0 }; break;
            case 5: ray.origin = calc<dZWXY, ZWXY<int> >(ray.intersectSquare<dZWXY>((dZWXY){ SPONGE_L, 0, 0, 0 }, SPONGE_L).second, true); res_normal = { 0, 0, 1, 0 }; break;
            case 6: ray.origin = calc<dWXYZ, WXYZ<int> >(ray.intersectSquare<dWXYZ>((dWXYZ){        0, 0, 0, 0 }, SPONGE_L).second, false); res_normal = { 0, 0, 0, 1 }; break;
            case 7: ray.origin = calc<dWXYZ, WXYZ<int> >(ray.intersectSquare<dWXYZ>((dWXYZ){ SPONGE_L, 0, 0, 0 }, SPONGE_L).second, true); res_normal = { 0, 0, 0, 1 }; //LAST BUG POSITION 20241104
        }
        if (blocks.size() >= init_blocks.size() + RELATIVE_FRACTAL_ITER) {
            res_ray = ray;
            return true;
        } else if (blocks.empty()) {
            res_ray.direction = ray.direction;
            return false;
        }
        // ray.origin.print();
        // putchar('\n');
        // if (ray.origin.x < 0 || ray.origin.x > SPONGE_L || ray.origin.y < 0 || ray.origin.y > SPONGE_L || ray.origin.z < 0 || ray.origin.z > SPONGE_L || ray.origin.w < 0 || ray.origin.w > SPONGE_L)
        //     std::cout << "bad\n";
    }
    return true;
}

//单次光线计算 从最外层块外射入情况
//三倍体代码（绷）
//return: 若撞到面了就返回 true
bool firstPropagate(Ray ray) {
    const bool x_hit = judgeHit(ray.origin.x, ray.direction.x);
    const bool y_hit = judgeHit(ray.origin.y, ray.direction.y);
    const bool z_hit = judgeHit(ray.origin.z, ray.direction.z);
    const bool w_hit = judgeHit(ray.origin.w, ray.direction.w);
    std::pair<double, V4d> x_hit_info = ray.intersectSquare<dXYZW>((dXYZW){ (double)x_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    std::pair<double, V4d> y_hit_info = ray.intersectSquare<dYZWX>((dYZWX){ (double)y_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    std::pair<double, V4d> z_hit_info = ray.intersectSquare<dZWXY>((dZWXY){ (double)z_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    std::pair<double, V4d> w_hit_info = ray.intersectSquare<dWXYZ>((dWXYZ){ (double)w_hit * SPONGE_L, 0.0, 0.0, 0.0 }, SPONGE_L);  // 待优化
    if (x_hit_info.first <= 0)  x_hit_info.first = INF; // 这三句在 judgeHit 里面好像判过了，可能可以删掉。
    if (y_hit_info.first <= 0)  y_hit_info.first = INF; // 这三句在 judgeHit 里面好像判过了，可能可以删掉。
    if (z_hit_info.first <= 0)  z_hit_info.first = INF; // 这三句在 judgeHit 里面好像判过了，可能可以删掉。
    if (w_hit_info.first <= 0)  w_hit_info.first = INF; // 这四句在 judgeHit 里面好像判过了，可能可以删掉。
    const double temp_at = std::min({ x_hit_info.first, y_hit_info.first, z_hit_info.first, w_hit_info.first });
    if (temp_at == INF) {
        res_ray.direction = ray.direction;
        return false;
    }

    // temp_blocks = blocks;   // 非光追可删
    
    if      (x_hit_info.first == temp_at)   { ray.origin = calc<dXYZW, XYZW<int> >(x_hit_info.second, x_hit); res_normal = { 1, 0, 0, 0 }; }
    else if (y_hit_info.first == temp_at)   { ray.origin = calc<dYZWX, YZWX<int> >(y_hit_info.second, y_hit); res_normal = { 0, 1, 0, 0 }; }
    else if (z_hit_info.first == temp_at)   { ray.origin = calc<dZWXY, ZWXY<int> >(z_hit_info.second, z_hit); res_normal = { 0, 0, 1, 0 }; }
    else if (w_hit_info.first == temp_at)   { ray.origin = calc<dWXYZ, WXYZ<int> >(w_hit_info.second, w_hit); res_normal = { 0, 0, 0, 1 }; }

    if (blocks.size() >= init_blocks.size() + RELATIVE_FRACTAL_ITER) {
        res_ray = ray;
        return true;
    }

    return propagate(ray);
}

/// @brief 光线追踪着色器（多次光线反射计算）
/// 现在暂时还是不带反射的版本
Color rayShader(Ray ray) {

    blocks = init_blocks;
    //memcpy(blocks.arr, init_blocks.arr, (init_blocks.size() + 2) * sizeof(Cube));   //这里好像 +1 就可以了，但是还是避免一下意外的溢出。
    //blocks.cursor = blocks.arr + init_blocks.size();
    //ray.direction.normalize();
    bool hit = blocks.empty() ? firstPropagate(ray) : propagate(ray);
    //std::cout << hit << std::endl;
    if (!hit)   return SKY_COLOR;
    // else if (res_normal.x == 1)
    //     return { 1.0, 0.0, 0.0 };
    // else if (res_normal.y == 1)
    //     return { 0.0, 1.0, 0.0 };
    // //else if (res_normal.z == 1)
    //     return { 0.0, 0.0, 1.0 };
    if (res_normal.w <= DIST_EPS)
        return { res_normal.x, res_normal.y, res_normal.z };
    return { 1.0, 1.0, 1.0 };   // w 方向暂时涂白色

}

// 计算距离趋向无穷时，此方向光线的亮度（假定光线在传播过程中不衰减）
Color calcLight(V4d direction) {
    return SKY_COLOR; //先不加方向光
}

// 生成 l~r 的随机数
double randLR(const double l, const double r) {
    return l + rng() / rng.max() * (r - l);
}

/// @brief 零到一之间的随机数
double rand01() {
    return rng() / (double)rng.max();
}

// 生成单位球体内的随机向量
// 已弃用
V4d randBall() {
    const double r = rand01();
    const double theta = 2 * M_PI * rand01();   // 俯仰角
    const double phi = M_PI * rand01();         // 方位角
    const double sin_phi = sin(phi);
    return (V4d){ sin_phi * cos(theta), sin_phi * sin(theta), cos(phi), 0 } * r;
}

// 适用于与坐标轴垂直的面，要求法向量标准化
V4d reflectVertical(V4d in, V4d normal) {
	if (abs(normal.x - 1) < DIST_EPS)
        return { -in.x, in.y, in.z, in.w };
    if (abs(normal.y - 1) < DIST_EPS)
        return { in.x, -in.y, in.z, in.w };
    if (abs(normal.z - 1) < DIST_EPS)
        return { in.x, in.y, -in.z, in.w };
    // else
        return { in.x, in.y, in.z, -in.w };
}

// 光线偏转
void deflectRay(V4d &direction, V4d normal) {
    V4d res;
    direction.normalize();
    if (dot(direction, normal) < 0)
        normal = normal.reverse();   // 如果法向量是扎向面里的，那么把它反转
    while (true) {
        res = direction + randBall() * SPONGE_ROUGHNESS;
        if (dot(res, normal) > 0) {
            direction = res;
            return; //一直循环直到产生在面外的反射光线为止。
        }
    }
}

// 计算光线经过传播、反射等之后的颜色
Color RTShader(Ray ray) {
    blocks = init_blocks;
    res_ray = ray;
    Color color = { 2, 2, 2 };
    double reflect_param = 1;   // 俄罗斯轮盘赌的方法里 根据反射次数算出的这个光线的权值
    do {
        if (propagate(res_ray)) {   // 碰上了
            res_ray.direction = reflectVertical(res_ray.direction, res_normal); // 反射
            deflectRay(res_ray.direction, res_normal);
            blocks = temp_blocks;
            color *= (Color){ 0.8, 0.8, 0.8 };  // 这里定义海绵颜色
        } else {
            color *= calcLight(res_ray.direction);
            return color * reflect_param;
        }
        reflect_param *= 1 / REFLECT_PROBABILITY;
    } while (rand01() < REFLECT_PROBABILITY);
    return { 0, 0, 0 };
}

#endif