#pragma once
#include "../math/MathUtils.h"
   struct  Shape
    {
        virtual float ComputeInertia(float mass) = 0;
    };
    struct Box : Shape
    {
        float width;
        float height;
        float ComputeInertia(float mass) override
        {
            return mass * (height * height + width * width) / 12;
        }
    };
    struct Circle : Shape
    {
        Vec2 center;
        float radius;
        float ComputeInertia(float mass) override
        {
            return PI * radius * radius * mass / 2;
        }
    };
    struct AABB 
    {
        Vec2 min;
        Vec2 max;
        AABB(Vec2 min, Vec2 max): min(min), max(max) {}

        
    };
    bool Intersects(const AABB& a, const AABB& b) 
    {
        return !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y);
    };
    bool CircleIntersects(const Circle& c1, const Circle& c2) 
    {
        Vec2 delta = c1.center - c2.center;
        float distanceSquared = delta.LenSq();
        float radiusSum = c1.radius + c2.radius;
        return distanceSquared <= radiusSum * radiusSum;
    };
    bool AABBvsCircle(const AABB& box, const Circle& circle) 
    {
        float closestX = std::clamp(circle.center.x, box.min.x, box.max.x);
        float closestY = std::clamp(circle.center.y, box.min.y, box.max.y);
        Vec2 closestPoint(closestX, closestY);
        Vec2 delta = circle.center - closestPoint;
        return delta.LenSq() <= circle.radius * circle.radius;
    };