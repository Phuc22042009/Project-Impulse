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
    float width = 0.0f;
    float height = 0.0f;
    Box() { type = ShapeType::Box; }
    Box(float w, float h) : width(w), height(h) { type = ShapeType::Box; }

    float ComputeInertia(float mass) const override
    {
        return mass * (height * height + width * width) / 12.0f;
    }
};
struct Circle : Shape
{
    Vec2 center;
    float radius = 0.0f;
    Circle() { type = ShapeType::Circle; }
    Circle(float r) : radius(r) { type = ShapeType::Circle; }
    
    float ComputeInertia(float mass) const override
    {
        return radius * radius * mass / 2.0f;
    }
};
struct AABB
{
    Vec2 min;
    Vec2 max;
    AABB(const Vec2 &min, const Vec2 &max) : min(min), max(max) {}
};
