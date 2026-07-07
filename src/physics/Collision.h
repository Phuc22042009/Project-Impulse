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
inline Manifold GetCircleManifold(const Circle &a, const Circle &b)
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
inline Manifold AABBvsCircleManifold(const AABB &box, const Circle &circle)
{
    Manifold manifold;
    manifold.colliding = false;

    float closestX = std::clamp(circle.center.x, box.min.x, box.max.x);
    float closestY = std::clamp(circle.center.y, box.min.y, box.max.y);

    Vec2 closestPoint(closestX, closestY);
    Vec2 delta = circle.center - closestPoint;

    float distanceSquared = delta.LenSq();

    if (distanceSquared < 1e-8f)
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
        if (minDist > distToBottom)
        {
            minDist = distToBottom;
            normal = Vec2(0, -1);
            manifold.contactPoint[0] = Vec2(circle.center.x, box.min.y);
        }
        if (minDist > distToTop)
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
inline void GetVertices(const RigidBody2D &body, const Box &box, Vec2 vertices[4])
{
    float hw = box.width * 0.5f;
    float hh = box.height * 0.5f;
    float cos = std::cos(body.angle);
    float sin = std::sin(body.angle);

    Vec2 localVertices[4] = {
        Vec2(-hw, -hh),
        Vec2(hw, -hh),
        Vec2(hw, hh),
        Vec2(-hw, hh)};
    for (int i = 0; i < 4; ++i)
    {
        vertices[i] = body.position + Vec2(localVertices[i].x * cos - localVertices[i].y * sin, localVertices[i].x * sin + localVertices[i].y * cos);
    }
}
inline void ProjectOBB(const Vec2 vertices[4], const Vec2 &axis, float &min, float &max)
{
    min = max = DotProduct(vertices[0], axis);
    for (int i = 1; i < 4; ++i)
    {
        float projection = DotProduct(vertices[i], axis);
        if (projection < min)
            min = projection;
        if (projection > max)
            max = projection;
    }
}
inline float TestAxis(const Vec2 verticesA[4], const Vec2 verticesB[4], const Vec2 &axis)
{
    float minA, maxA, minB, maxB;
    ProjectOBB(verticesA, axis, minA, maxA);
    ProjectOBB(verticesB, axis, minB, maxB);

    float overlap = std::min(maxA, maxB) - std::max(minA, minB);
    return overlap;
}
inline Vec2 FindContactPoint(const Vec2 verticesB[4], const RigidBody2D &bodyA, const Box &boxA)
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
        float minDistSq = std::numeric_limits<float>::max();
        for (int i = 0; i < 4; ++i)
        {
            Vec2 d = verticesB[i] - bodyA.position;
            float distSq = d.LenSq();
            if (distSq < minDistSq)
            {
                minDist = distSq;
                contactPoint = verticesB[i];
            }
        }
    }
    return contactPoint;
}
inline int FindContactPoints(const Vec2 verticlesB[4], const RigidBody2D &bodyA, const Box &boxA, Vec2 contactPoints[2])
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
inline void ReduceContactPoints(Manifold &manifold)
{
    if (manifold.contactCount <= 1)
        return;

    const float closestDistSq = 1.0f;
    for (int i = 0; i < manifold.contactCount; ++i)
    {
        for (int j = i + 1; j < manifold.contactCount; ++j)
        {
            if ((manifold.contactPoint[i] - manifold.contactPoint[j]).LenSq() < closestDistSq)
            {
                manifold.contactPoint[j] = manifold.contactPoint[manifold.contactCount - 1];
                manifold.contactCount--;
                j--;
            }
        }
    }
    if (manifold.contactCount > 2)
    {
        int indexA = 0, indexB = 1;
        float maxDistSq = (manifold.contactPoint[0] - manifold.contactPoint[1]).LenSq();
        for (int i = 0; i < manifold.contactCount; ++i)
        {
            for (int j = i + 1; j < manifold.contactCount; ++j)
            {
                float distSq = (manifold.contactPoint[i] - manifold.contactPoint[j]).LenSq();
                if (distSq > maxDistSq)
                {
                    maxDistSq = distSq;
                    indexA = i;
                    indexB = j;
                }
            }
        }
        Vec2 pA = manifold.contactPoint[indexA];
        Vec2 pB = manifold.contactPoint[indexB];

        manifold.contactPoint[0] = pA;
        manifold.contactPoint[1] = pB;
        manifold.contactCount = 2;
    }
}
inline Manifold OBBvsOBBManifold(const RigidBody2D &bodyA, const Box &boxA, const RigidBody2D &bodyB, const Box &boxB)
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
        (verticesB[3] - verticesB[0]).Perp().Normalized()};

    float minOverlap = std::numeric_limits<float>::max();
    Vec2 smallestAxis;

    for (const Vec2 &axis : axes)
    {
        float overlap = TestAxis(verticesA, verticesB, axis);
        if (overlap <= 0)
            return manifold;
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
        ReduceContactPoints(manifold);
    }
    return manifold;
}
inline Manifold OBBvsCircleManifold(const RigidBody2D &bodyA, const Box &boxA, const RigidBody2D &bodyB, const Circle &circleB)
{
    Manifold manifold;
    manifold.colliding = false;

    float cosA = std::cos(bodyA.angle);
    float sinA = std::sin(bodyA.angle);

    Vec2 axisX = Vec2(cosA, sinA);
    Vec2 axisY = Vec2(-sinA, cosA);

    Vec2 d = circleB.center - bodyA.position;

    float localX = DotProduct(d, axisX);
    float localY = DotProduct(d, axisY);

    float hw = boxA.width * 0.5f;
    float hh = boxA.height * 0.5f;

    float clampedX = std::clamp(localX, -hw, hw);
    float clampedY = std::clamp(localY, -hh, hh);

    Vec2 closestPoint = bodyA.position + axisX * clampedX + axisY * clampedY;
    Vec2 delta = circleB.center - closestPoint;
    float distSq = delta.LenSq();
    bool centerInside = (std::abs(localX) < hw && std::abs(localY) < hh);
    if (centerInside)
    {
        float overlapX = hw - std::abs(localX);
        float overlapY = hh - std::abs(localY);

        if (overlapX < overlapY)
        {
            manifold.penetration = circleB.radius + overlapX;
            manifold.normal = (localX > 0) ? axisX : axisX * -1.0f;
        }
        else
        {
            manifold.penetration = circleB.radius + overlapY;
            manifold.normal = (localY > 0) ? axisY : axisY * -1.0f;
        }
        manifold.colliding = true;
        manifold.contactPoint[0] = closestPoint;
        manifold.contactCount = 1;
        return manifold;
    }

    if (distSq < circleB.radius * circleB.radius)
    {
        float distance = sqrt(distSq);
        manifold.colliding = true;
        manifold.penetration = circleB.radius - distance;
        manifold.normal = delta / distance;
        manifold.contactPoint[0] = closestPoint;
        manifold.contactCount = 1;
    }
    return manifold;
}
inline Manifold GetManifold(RigidBody2D *a, RigidBody2D *b)
{
    if (!a || !b || !a->shape || !b->shape)
    {
        Manifold empty;
        empty.colliding = false;
        return empty;
    }

    if (a->shape->type == ShapeType::Box && b->shape->type == ShapeType::Box)
    {
        return OBBvsOBBManifold(*a, static_cast<Box &>(*a->shape.get()), *b, static_cast<Box &>(*b->shape.get()));
    }

    if (a->shape->type == ShapeType::Circle && b->shape->type == ShapeType::Circle)
    {
        Circle circA = *static_cast<Circle *>(a->shape.get());
        Circle circB = *static_cast<Circle *>(b->shape.get());
        circA.center = a->position;
        circB.center = b->position;
        return GetCircleManifold(circA, circB);
    }

    if (a->shape->type == ShapeType::Box && b->shape->type == ShapeType::Circle)
    {
        Circle circB = *static_cast<Circle *>(b->shape.get());
        circB.center = b->position;
        Box boxA = *static_cast<Box *>(a->shape.get());
        Manifold manifold = OBBvsCircleManifold(*a, boxA, *b, circB);
        manifold.normal = manifold.normal * -1.0f;
        return manifold;
    }

    if (a->shape->type == ShapeType::Circle && b->shape->type == ShapeType::Box)
    {
        Circle circA = *static_cast<Circle *>(a->shape.get());
        circA.center = a->position;
        Box boxB = *static_cast<Box *>(b->shape.get());
        Manifold manifold = OBBvsCircleManifold(*b, boxB, *a, circA);
        manifold.normal = manifold.normal;
        return manifold;
    }

    Manifold empty;
    empty.colliding = false;
    return empty;
}
