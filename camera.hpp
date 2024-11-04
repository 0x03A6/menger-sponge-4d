#pragma once
#ifndef CAMREA_HPP
#define CAMERA_HPP

#include "vector.hpp"
#include "sponge.hpp"

struct Camera {

    V4d pos;
    V4d front, right, head, somewhere;  // somewhere: 摄像机对应第四维度指向的空间中的方向
    V4d relative_pos;

    Camera() {
        front = INIT_CAMERA_FRONT;
        head = INIT_CAMERA_HEAD;
        pos = INIT_CAMERA_POS;
        somewhere = INIT_CAMERA_SOMEWHERE;
        front.normalize();
        head.normalize();
        somewhere.normalize();
        //pos.normalize();
        right = cross(front, head, somewhere); // TODO
    }
    
    //坐标系转换相关函数：
    V4d worldToCam(V4d a) const {
        return { dot(a, front), dot(a, right), dot(a, head), dot(a, somewhere) };
    }
    V4d camToWorld(V4d a) const {
        return worldToCam(a);
    }
    //将二维平面上的坐标转化为像素
    static Pixel screenToPixel(V2d point) {
        return { -(int)(point.y * SCALE) + H / 2, (int)(point.x * SCALE) + W / 2 };
    }
    //将像素转化为二维平面上的坐标
    static V2d pixelToScreen(Pixel pixel) {
        return { (double)(pixel.c - W / 2) / SCALE, (double)(-pixel.l + H / 2) / SCALE };
    }
    static V2d pixelToScreen(const double l, const double c) {
        return { (c - W / 2) / SCALE, (-l + H / 2) / SCALE };
    }
    V4d screenToSpace(V2d point) const {
        return relative_pos + front * CAMERA_AXIS + right * point.x + head * point.y;
    }
    V4d pixelToSpace(Pixel px) const {
        return screenToSpace(pixelToScreen(px));
    }
    V4d pixelToSpace(const double l, const double c) const {
        return screenToSpace(pixelToScreen(l, c));
    }

    void moveLeft(const double delta) {
        pos -= right * delta * move_velocity;
    }
    void moveRight(const double delta) {
        pos += right * delta * move_velocity;
    }
    void moveUp(const double delta) {
        pos += head * delta * move_velocity;
    }
    void moveDown(const double delta) {
        pos -= head * delta * move_velocity;
    }
    void moveForward(const double delta) {
        pos += front * delta * move_velocity;
    }
    void moveBackward(const double delta) {
        pos -= front * delta * move_velocity;
    }
    //左右摇头，正数往左摇
    void rotateLR(const double delta) {
        right += front * delta * ROTATE_VELOCITY;
        right.normalize();
        front = cross(head, right, somewhere);
        //front.normalize();
    }
    //上下摇头，正数往上摇
    void rotateUD(const double delta) {
        front += head * delta * ROTATE_VELOCITY;
        front.normalize();
        head = cross(right, front, somewhere);
        //head.normalize();
    }
    void rotateWPositive(const double delta) {
        front += somewhere * delta * ROTATE_VELOCITY * 0.1;
        front.normalize();
        somewhere = cross(front, head, right).reverse();
    }
    void rotateWNegative(const double delta) {
        front -= somewhere * delta * ROTATE_VELOCITY * 0.1;
        front.normalize();
        somewhere = cross(front, head, right).reverse();
    }
};

struct CamInfo {
        V4d pos;
        V4d front, right, head, somewhere;
        CamInfo(Camera c) {
            pos = c.pos;
            front = c.front;
            right = c.right;
            head = c.head;
            somewhere = c.somewhere;
        }
        CamInfo(const V4d p, const V4d f, const V4d r, const V4d h, const V4d s) {
            pos = p;
            front = f;
            right = r;
            head = h;
            somewhere = s;
        }
        CamInfo() {}
    };

#endif