#pragma once
#include "shape.h"
#include "../math/Vec2.h"
bool Intersects(const AABB& a, const AABB& b) 
    {
        return !(a.max.x < b.min.x || a.min.x > b.max.x || a.max.y < b.min.y || a.min.y > b.max.y);
    };
bool GetCircleManifold(const Circle& c1, const Circle& c2, Manifold& manifold) 
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
struct Manifold
    {
        bool colliding;
        Vec2 normal;
        float penetration;
    };
Manifold GetCircleManifold(const Circle& a, const Circle& b) 
    {
        Manifold manifold;
        manifold.colliding = false;

        Vec2 delta = a.center - b.center;

        float distance = delta.Len();
        float radiusSum = a.radius + b.radius;

        if (distance < radiusSum) 
        {
                manifold.colliding = true;
                manifold.penetration = radiusSum - distance;
                manifold.normal = distance > 1e-8f ? delta / distance : Vec2(1, 0); 
        }
        return manifold;
    };
Manifold AABBvsCircleManifold(const AABB& box, const Circle& circle) 
    {
        Manifold manifold;
        manifold.colliding = false;

        float closestX = std::clamp(circle.center.x, box.min.x, box.max.x);
        float closestY = std::clamp(circle.center.y, box.min.y, box.max.y);

        Vec2 closestPoint(closestX, closestY);
        Vec2 delta = circle.center - closestPoint;

        float distanceSquared = delta.LenSq();

         if(distanceSquared < 1e-8f) 
        {
            float distToLeft = std::abs(circle.center.x - box.min.x);
            float distToRight = std::abs(circle.center.x - box.max.x);
            float distToBottom = std::abs(circle.center.y - box.min.y);
            float distToTop = std::abs(circle.center.y - box.max.y);

            float minDist = distToLeft;
            Vec2 normal = Vec2(-1, 0);
            if (minDist > distToRight) 
            {
                minDist = distToRight;
                normal = Vec2(1, 0);
            }
            if(minDist > distToBottom) 
            {
                minDist = distToBottom;
                normal = Vec2(0, -1);
            }
            if(minDist > distToTop) 
            {
                minDist = distToTop;
                normal = Vec2(0, 1);
            }

            manifold.colliding = true;
            manifold.penetration = circle.radius + minDist;
            manifold.normal = normal;
            return manifold;
        }

        if (distanceSquared < circle.radius * circle.radius) 
        {
            float distance = sqrt(distanceSquared);
            manifold.colliding = true;
            manifold.penetration = circle.radius - distance;
            manifold.normal = distance > 1e-8f ? delta / distance : Vec2(0, 1); 
        }
        return manifold;
    };
Manifold AABBvsAABBManifold(const AABB& a, const AABB& b) 
    {
        Manifold manifold;
        manifold.colliding = false;

        Vec2 centerA = (a.min + a.max) * 0.5f;
        Vec2 centerB = (b.min + b.max) * 0.5f;

        Vec2 centerAtoCenterB = centerB - centerA;

        Vec2 halfExtentsA = (a.max - a.min) * 0.5f;
        Vec2 halfExtentsB = (b.max - b.min) * 0.5f;

        float overlapX = halfExtentsA.x + halfExtentsB.x - std::abs(centerAtoCenterB.x);
        float overlapY = halfExtentsA.y + halfExtentsB.y - std::abs(centerAtoCenterB.y);
        if (overlapX > 0 && overlapY > 0) 
        {
            manifold.colliding = true;
            if (overlapX < overlapY) 
            {
                manifold.penetration = overlapX;
                manifold.normal = (centerAtoCenterB.x < 0) ? Vec2(-1, 0) : Vec2(1, 0);
            } 
            else 
            {
                manifold.penetration = overlapY;
                manifold.normal = (centerAtoCenterB.y < 0) ? Vec2(0, -1) : Vec2(0, 1);
            }
        }
        return manifold;
    };
    