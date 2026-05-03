#pragma once
#include "shape.h"
#include "../math/Vec2.h"
#include "../physics/RigidBody2D.h"
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
        Vec2 contactPoint;
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
                manifold.contactPoint = b.center + manifold.normal * b.radius;
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
            manifold.contactPoint = Vec2(box.min.x, circle.center.y);
            if (minDist > distToRight) 
            {
                minDist = distToRight;
                normal = Vec2(1, 0);
                manifold.contactPoint = Vec2(box.max.x, circle.center.y);
            }
            if(minDist > distToBottom) 
            {
                minDist = distToBottom;
                normal = Vec2(0, -1);
                manifold.contactPoint = Vec2(circle.center.x, box.min.y);
            }
            if(minDist > distToTop) 
            {
                minDist = distToTop;
                normal = Vec2(0, 1);
                manifold.contactPoint = Vec2(circle.center.x, box.max.y);
            }

            manifold.colliding = true;
            manifold.penetration = circle.radius - minDist;
            manifold.normal = normal;
            return manifold;
        }

        if (distanceSquared < circle.radius * circle.radius) 
        {
            float distance = sqrt(distanceSquared);
            manifold.colliding = true;
            manifold.penetration = circle.radius - distance;
            manifold.normal = distance > 1e-8f ? delta / distance : Vec2(0, 1); 
            manifold.contactPoint = closestPoint;
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
            manifold.contactPoint = centerA + Vec2(manifold.normal.x * halfExtentsA.x, manifold.normal.y * halfExtentsA.y);
        }
        return manifold;
    };
void ResolveCollision(RigidBody2D& a, RigidBody2D& b, const Manifold& manifold)
    {
        Vec2 rA = manifold.contactPoint - a.position;
        Vec2 rB = manifold.contactPoint - b.position;
        Vec2 vContactA = a.velocity + Vec2(-a.angularVelocity * rA.y, a.angularVelocity * rA.x);
        Vec2 vContactB = b.velocity + Vec2(-b.angularVelocity * rB.y, b.angularVelocity * rB.x);
        Vec2 relativeVelocity = vContactA - vContactB;

        float vReldotN = DotProduct(relativeVelocity, manifold.normal);
        if (vReldotN >= 0) return;

        float e = std::min(a.restitution, b.restitution);
        float denominator = a.invMass + b.invMass + CrossProduct(rA, manifold.normal) * a.invInertia * CrossProduct(rA, manifold.normal) + CrossProduct(rB, manifold.normal) * b.invInertia * CrossProduct(rB, manifold.normal);
        float j = -(1 + e) * vReldotN / denominator; 

        a.velocity = a.velocity + manifold.normal * (j * a.invMass);
        a.angularVelocity += CrossProduct(rA, manifold.normal * j) * a.invInertia;

        b.velocity = b.velocity - manifold.normal * (j * b.invMass);
        b.angularVelocity -= CrossProduct(rB, manifold.normal * j) * b.invInertia;

        Vec2 tangent = relativeVelocity - manifold.normal * vReldotN;
        if (tangent.Len() < 1e-8f) return;
        Vec2 tangentDir = tangent / tangent.Len();

        float jtDenominator = a.invMass + b.invMass + CrossProduct(rA, tangentDir) * a.invInertia * CrossProduct(rA, tangentDir) + CrossProduct(rB, tangentDir) * b.invInertia * CrossProduct(rB, tangentDir);
        if (jtDenominator < 1e-8f) return;
        float jt = -DotProduct(relativeVelocity, tangentDir) / jtDenominator;
        float mu = (a.staticFriction + b.staticFriction) * 0.5f;

        if (std::abs(jt) < j * mu) 
        {
            a.velocity = a.velocity + tangentDir * (jt * a.invMass);
            a.angularVelocity += CrossProduct(rA, tangentDir * jt) * a.invInertia;
            b.velocity = b.velocity - tangentDir * (jt * b.invMass);
            b.angularVelocity -= CrossProduct(rB, tangentDir * jt) * b.invInertia;
        }
        else 
        {
            a.velocity = a.velocity + tangentDir * (-j * mu * a.invMass);
            a.angularVelocity += CrossProduct(rA, tangentDir * -j * mu) * a.invInertia;
            b.velocity = b.velocity - tangentDir * (-j * mu * b.invMass);
            b.angularVelocity -= CrossProduct(rB, tangentDir   * -j * mu) * b.invInertia;
        }
    };
void PositionalCorrection(RigidBody2D& a, RigidBody2D& b, const Manifold& manifold)
    {
        const float percent = 0.2f; 
        const float slop = 0.01f; 
        float correctionMagnitude = std::max(manifold.penetration - slop, 0.0f) / (a.invMass + b.invMass) * percent;
        Vec2 correction = manifold.normal * correctionMagnitude;
        a.position = a.position + correction * a.invMass;
        b.position = b.position - correction * b.invMass;
    }