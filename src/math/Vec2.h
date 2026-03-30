#pragma once
#include <math.h>
struct Vec2 
{
    float x;
    float y;
    Vec2(): x(0), y(0) {}
    Vec2(float x, float y): x(x), y(y) {} 
    
    float LenSq() const 
    { 
        return x * x + y * y; 
    }
    float Len() const 
    { 
        return std::sqrt(LenSq()); 
    }
    float DotProduct(Vec2 a, Vec2 b)
    {
        return a.x * b.x + a.y * b.y;
    }
    Vec2 operator+(const Vec2& other) const
    {
        return Vec2(x + other.x, y + other.y);
    }
    Vec2 operator-(const Vec2& other) const 
    {
        return Vec2(x - other.x, y - other.y);
    }
    Vec2 operator*(float s) const {
        return Vec2(x * s, y * s);
    }
    Vec2 operator/(float s) const {
        return Vec2( x / s, y / s);
    }
    Vec2 Perp() const { return {-y, x}; }
   
    Vec2 Normalized() const {
        float l = Len();
        if (l < 1e-8f) return {0, 0};
        return {x / l, y / l};
    }
     Vec2 Rotated(float angle) const {
        float c = std::cos(angle);
        float s = std::sin(angle);
        return { x * c - y * s,  x * s + y * c };
    }
};