#pragma once
#include <cmath>
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
        return sqrt(LenSq()); 
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
        float c = cos(angle);
        float s = sin(angle);
        return { x * c - y * s,  x * s + y * c };
    }
};
inline float DotProduct(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline float CrossProduct(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}