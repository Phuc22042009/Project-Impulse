#pragma once

struct Vec2 
{
    float x;
    float y;
    Vec2(): x(0), y(0) {}
    Vec2(float x, float y): x(x), y(y) {} 
    
    Vec2 operator+(const Vec2& other) const
    {
        return Vec2(x + other.x, y + other.y);
    }
    Vec2 operator-(const Vec2& other) const 
    {
        return Vec2(x - other.x, y - other.y);
    }
    float DotProduct(Vec2 a, Vec2 b)
    {
        return a.x * b.x + a.y * b.y;
    }
};