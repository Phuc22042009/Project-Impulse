#pragma once
#include "Shape.h"
#include "../math/Vec2.h"
#include "../physics/RigidBody2D.h"
struct Manifold
    {
        bool colliding = false;
        Vec2 normal;
        float penetration = 0;
        Vec2 contactPoint[4];
        int contactCount = 0;
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
            manifold.contactPoint[0] = b.center + manifold.normal * b.radius;
            manifold.contactCount = 1;
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
            manifold.contactPoint[0] = Vec2(box.min.x, circle.center.y);

            if (minDist > distToRight) 
            {
                minDist = distToRight;
                normal = Vec2(1, 0);
                manifold.contactPoint[0] = Vec2(box.max.x, circle.center.y);
            }
            if(minDist > distToBottom) 
            {
                minDist = distToBottom;
                normal = Vec2(0, -1);
                manifold.contactPoint[0] = Vec2(circle.center.x, box.min.y);
            }
            if(minDist > distToTop) 
            {
                minDist = distToTop;
                normal = Vec2(0, 1);
                manifold.contactPoint[0] = Vec2(circle.center.x, box.max.y);
            }

            manifold.colliding = true;
            manifold.penetration = circle.radius - minDist;
            manifold.normal = normal;
            manifold.contactCount = 1;
            return manifold;
        }

        if (distanceSquared < circle.radius * circle.radius) 
        {
            float distance = sqrt(distanceSquared);
            manifold.colliding = true;
            manifold.penetration = circle.radius - distance;
            manifold.normal = distance > 1e-8f ? delta / distance : Vec2(0, 1); 
            manifold.contactPoint[0] = closestPoint;
            manifold.contactCount = 1;
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
                manifold.normal = (centerAtoCenterB.x > 0) ? Vec2(-1, 0) : Vec2(1, 0);
            } 
            else 
            {
                manifold.penetration = overlapY;
                manifold.normal = (centerAtoCenterB.y > 0) ? Vec2(0, -1) : Vec2(0, 1);
            }
            manifold.contactPoint[0] = centerA + Vec2(manifold.normal.x * halfExtentsA.x, manifold.normal.y * halfExtentsA.y);
            manifold.contactCount = 1;
        }
        return manifold;
    };
void GetVertices(const RigidBody2D& body, const Box& box, Vec2 vertices[4]) 
    {
        float hw = box.width * 0.5f;
        float hh = box.height * 0.5f;
        float cos = std::cos(body.angle);
        float sin = std::sin(body.angle);

        Vec2 localVertices[4] = {
            Vec2(-hw, -hh),
            Vec2(hw, -hh),
            Vec2(hw, hh),
            Vec2(-hw, hh)
        };
        for (int i = 0; i < 4; ++i) 
        {
            vertices[i] = body.position + Vec2(localVertices[i].x * cos - localVertices[i].y * sin, localVertices[i].x * sin + localVertices[i].y * cos);
        }
    }
void ProjectOBB(const Vec2 vertices[4], const Vec2& axis, float& min, float& max) 
    {
        min = max = DotProduct(vertices[0], axis);
        for (int i = 1; i < 4; ++i) 
        {
            float projection = DotProduct(vertices[i], axis);
            if (projection < min) min = projection;
            if (projection > max) max = projection;
        }
    }
float TestAxis(const Vec2 verticesA[4], const Vec2 verticesB[4], const Vec2& axis) 
    {
        float minA, maxA, minB, maxB;
        ProjectOBB(verticesA, axis, minA, maxA);
        ProjectOBB(verticesB, axis, minB, maxB);

        float overlap = std::min(maxA, maxB) - std::max(minA, minB);
        return overlap;
    }
Vec2 FindContactPoint(const Vec2 verticesB[4], const RigidBody2D& bodyA, const Box& boxA)
    {
        Vec2 contactPoint;
        float minDist = std::numeric_limits<float>::max();

        float cosA = std::cos(bodyA.angle);
        float sinA = std::sin(bodyA.angle);
        Vec2 axisA1 = Vec2(cosA, sinA);
        Vec2 axisA2 = Vec2(-sinA, cosA);

        float halfWidth = boxA.width * 0.5f;
        float halfHeight = boxA.height * 0.5f;

        for (int i = 0; i < 4; ++i) 
        {
            Vec2 d = verticesB[i] - bodyA.position;
            float projX = DotProduct(d, axisA1);
            float projY = DotProduct(d, axisA2);

            if (std::abs(projX) <= halfWidth && std::abs(projY) <= halfHeight) 
            {
                float depthX = halfWidth - std::abs(projX);
                float depthY = halfHeight - std::abs(projY);
                float depth = std::min(depthX, depthY);

                if (depth < minDist) 
                {
                    minDist = depth;
                    contactPoint = verticesB[i];
                }
            }
        }

        if (minDist == std::numeric_limits<float>::max()) 
        {
            for (int i = 0; i < 4; ++i) 
            {
                Vec2 d = verticesB[i] - bodyA.position;
                float distSq = d.LenSq();
                if (distSq < minDist) 
                {
                    minDist = distSq;
                    contactPoint = verticesB[i];
                }
            }
        }
        return contactPoint;
    }
int FindContactPoints(const Vec2 verticlesB[4], const RigidBody2D& bodyA, const Box& boxA, Vec2 contactPoints[2]) 
    {
        float cosA = std::cos(bodyA.angle);
        float sinA = std::sin(bodyA.angle);
        Vec2 axisA1 = Vec2(cosA, sinA);
        Vec2 axisA2 = Vec2(-sinA, cosA);

        float halfWidth = boxA.width * 0.5f;
        float halfHeight = boxA.height * 0.5f;

        int count = 0;
        for (int i = 0; i < 4 && count < 2; ++i) 
        {
            Vec2 d = verticlesB[i] - bodyA.position;
            float projX = DotProduct(d, axisA1);
            float projY = DotProduct(d, axisA2);
            if (std::abs(projX) <= halfWidth && std::abs(projY) <= halfHeight) 
            {
                contactPoints[count++] = verticlesB[i];
            }
        }
        return count;
    }
Manifold OBBvsOBBManifold(const RigidBody2D& bodyA, const Box& boxA, const RigidBody2D& bodyB, const Box& boxB) 
    {
        Manifold manifold;
        manifold.colliding = false;

        Vec2 verticesA[4], verticesB[4];
        GetVertices(bodyA, boxA, verticesA);
        GetVertices(bodyB, boxB, verticesB);

        Vec2 axes[4] = {
            (verticesA[1] - verticesA[0]).Perp().Normalized(),
            (verticesA[3] - verticesA[0]).Perp().Normalized(),
            (verticesB[1] - verticesB[0]).Perp().Normalized(),
            (verticesB[3] - verticesB[0]).Perp().Normalized()
        };

        float minOverlap = std::numeric_limits<float>::max();
        Vec2 smallestAxis;

        for (const Vec2& axis : axes) 
        {
            float overlap = TestAxis(verticesA, verticesB, axis);
            if (overlap <= 0) return manifold;
            if (overlap < minOverlap) 
            {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }

        manifold.colliding = true;
        manifold.penetration = minOverlap;
        
        Vec2 centerAtocenterB = bodyB.position - bodyA.position;
        if (DotProduct(centerAtocenterB, smallestAxis) > 0)
        {
            manifold.normal = smallestAxis * -1.0f;
        }
        else
        {
            manifold.normal = smallestAxis;
        }

        Vec2 allPoints[4];
        int totalCount = 0;

        Vec2 pointsBA[2];
        int countBA = FindContactPoints(verticesB, bodyA, boxA, pointsBA);
        for (int i = 0; i < countBA; i++)
            allPoints[totalCount++] = pointsBA[i];

        Vec2 pointsAB[2];
        int countAB = FindContactPoints(verticesA, bodyB, boxB, pointsAB);
        for (int i = 0; i < countAB && totalCount < 4; i++)
            allPoints[totalCount++] = pointsAB[i];

        if (totalCount == 0) 
        {
            manifold.contactPoint[0] = FindContactPoint(verticesB, bodyA, boxA);
            manifold.contactCount = 1;
        }
        else
        {
            manifold.contactCount = totalCount;
            for (int i = 0; i < totalCount; i++)
                manifold.contactPoint[i] = allPoints[i];
        }
        return manifold;
    }
Manifold GetManifold(RigidBody2D& a, RigidBody2D& b)
{
    Box* boxA  = dynamic_cast<Box*>(a.shape);
    Box* boxB  = dynamic_cast<Box*>(b.shape);
    Circle* circA = dynamic_cast<Circle*>(a.shape);
    Circle* circB = dynamic_cast<Circle*>(b.shape);

    if (boxA && boxB)
    {
        return OBBvsOBBManifold(a, *boxA, b, *boxB);
    }

    if (circA && circB)
    {
        circA->center = a.position;
        circB->center = b.position;
        return GetCircleManifold(*circA, *circB);
    }

    if (boxA && circB)
    {
        AABB aabbA(a.position - Vec2(boxA->width*0.5f, boxA->height*0.5f), a.position + Vec2(boxA->width*0.5f, boxA->height*0.5f));
        circB->center = b.position;
        return AABBvsCircleManifold(aabbA, *circB);
    }

    if (circA && boxB)
    {
        AABB aabbB(b.position - Vec2(boxB->width*0.5f, boxB->height*0.5f), b.position + Vec2(boxB->width*0.5f, boxB->height*0.5f));
        circA->center = a.position;
        Manifold m = AABBvsCircleManifold(aabbB, *circA);
        m.normal = m.normal * -1.0f;
        return m;
    }

    Manifold empty; empty.colliding = false;
    return empty;
}

