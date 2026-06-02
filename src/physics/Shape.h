#pragma once
#include "../math/MathUtils.h"
#include "../math/Vec2.h"
   struct Shape
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
            return radius * radius * mass / 2;
        }
    };
    struct AABB 
    {
        Vec2 min;
        Vec2 max;
        AABB(Vec2 min, Vec2 max): min(min), max(max) {}
    };
    