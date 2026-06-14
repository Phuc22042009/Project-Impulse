#pragma once
#include "../math/MathUtils.h"
#include "../math/Vec2.h"

enum class ShapeType
{
    Box,
    Circle
};
struct Shape
{
    ShapeType type;
    virtual float ComputeInertia(float mass) const = 0;
    virtual ~Shape() = default;
};
struct Box : Shape
{
    float width;
    float height;
    Box() {type = ShapeType::Box;}
    float ComputeInertia(float mass) const override
    {
        return mass * (height * height + width * width) / 12;
    }
};
struct Circle : Shape
{
    Vec2 center;
    float radius;
    Circle() {type = ShapeType::Circle;}
    float ComputeInertia(float mass) const override
    {
        return radius * radius * mass / 2;
    }
};
struct AABB 
{
    Vec2 min;
    Vec2 max;
    AABB(Vec2 min, Vec2 max): min(min), max(max) {}
};
    