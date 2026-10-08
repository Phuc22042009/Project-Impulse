#include "Collision.h"
#include "../math/MathUtils.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cassert>

Manifold GetCircleManifold(const Circle &a, const Circle &b)
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
        manifold.contactPenetration[0] = manifold.penetration;
        manifold.contactCount = 1;
    }

    return manifold;
};
Manifold AABBvsCircleManifold(const AABB &box, const Circle &circle)
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
void GetVertices(const RigidBody2D &body, const Box &box, Vec2 vertices[4])
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
void ProjectOBB(const Vec2 vertices[4], const Vec2 &axis, float &min, float &max)
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
float TestAxis(const Vec2 verticesA[4], const Vec2 verticesB[4], const Vec2 &axis)
{
    float minA, maxA, minB, maxB;
    ProjectOBB(verticesA, axis, minA, maxA);
    ProjectOBB(verticesB, axis, minB, maxB);

    float overlap = std::min(maxA, maxB) - std::max(minA, minB);
    return overlap;
}
struct ClipVertex
{
    Vec2 v;
};

static int ClipSegmentToLine(ClipVertex vOut[2], const ClipVertex vIn[2], const Vec2 &normal, float offset)
{
    int numOut = 0;
    float d0 = DotProduct(normal, vIn[0].v) - offset;
    float d1 = DotProduct(normal, vIn[1].v) - offset;

    if (d0 >= 0.0f) vOut[numOut++] = vIn[0];
    if (d1 >= 0.0f) vOut[numOut++] = vIn[1];

    if (d0 * d1 < 0.0f)
    {
        float alpha = d0 / (d0 - d1);
        vOut[numOut++].v = vIn[0].v + (vIn[1].v - vIn[0].v) * alpha;
    }
    return numOut;
}

struct FeatureEdge
{
    Vec2 v1;
    Vec2 v2;
    Vec2 normal;
};

static FeatureEdge FindBestEdge(const Vec2 vertices[4], const Vec2 &targetDir)
{
    FeatureEdge bestEdge;
    bestEdge.v1 = vertices[0];
    bestEdge.v2 = vertices[1];
    bestEdge.normal = Vec2(0, -1);
    float maxDot = -std::numeric_limits<float>::max();

    for (int i = 0; i < 4; ++i)
    {
        Vec2 vA = vertices[i];
        Vec2 vB = vertices[(i + 1) % 4];
        Vec2 edge = vB - vA;
        
        Vec2 normal = Vec2(edge.y, -edge.x).Normalized();

        float dot = DotProduct(normal, targetDir);
        if (dot > maxDot)
        {
            maxDot = dot;
            bestEdge.v1 = vA;
            bestEdge.v2 = vB;
            bestEdge.normal = normal;
        }
    }
    return bestEdge;
}

Manifold OBBvsOBBManifold(const RigidBody2D &bodyA, const Box &boxA, const RigidBody2D &bodyB, const Box &boxB)
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
        if (overlap <= 0.0f)
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
    Vec2 normalBA;
    if (DotProduct(centerAtocenterB, smallestAxis) > 0.0f)
    {
        normalBA = smallestAxis * -1.0f;
    }
    else
    {
        normalBA = smallestAxis;
    }
    manifold.normal = normalBA;

    FeatureEdge edgeA = FindBestEdge(verticesA, normalBA * -1.0f);
    float dotA = DotProduct(edgeA.normal, normalBA * -1.0f);

    FeatureEdge edgeB = FindBestEdge(verticesB, normalBA);
    float dotB = DotProduct(edgeB.normal, normalBA);

    FeatureEdge refEdge, incEdge;
    if (dotA >= dotB)
    {
        refEdge = edgeA;
        incEdge = FindBestEdge(verticesB, refEdge.normal * -1.0f);
    }
    else
    {
        refEdge = edgeB;
        incEdge = FindBestEdge(verticesA, refEdge.normal * -1.0f);
    }

    Vec2 refTangent = (refEdge.v2 - refEdge.v1).Normalized();

    ClipVertex incVertices[2] = {{incEdge.v1}, {incEdge.v2}};
    ClipVertex clipped1[2];
    int count1 = ClipSegmentToLine(clipped1, incVertices, refTangent, DotProduct(refTangent, refEdge.v1));
    if (count1 < 2)
        return manifold;

    ClipVertex clipped2[2];
    int count2 = ClipSegmentToLine(clipped2, clipped1, refTangent * -1.0f, DotProduct(refTangent * -1.0f, refEdge.v2));
    if (count2 == 0)
        return manifold;

    for (int i = 0; i < count2; ++i)
    {
        float sep = DotProduct(refEdge.normal, clipped2[i].v - refEdge.v1);
        if (sep <= 0.0f)
        {
            manifold.contactPoint[manifold.contactCount] = clipped2[i].v;
            manifold.contactPenetration[manifold.contactCount] = -sep;
            manifold.contactCount++;
        }
    }

    if (manifold.contactCount == 0 && count2 > 0)
    {
        int bestIdx = 0;
        float minSep = DotProduct(refEdge.normal, clipped2[0].v - refEdge.v1);
        for (int i = 1; i < count2; ++i)
        {
            float s = DotProduct(refEdge.normal, clipped2[i].v - refEdge.v1);
            if (s < minSep)
            {
                minSep = s;
                bestIdx = i;
            }
        }
        manifold.contactPoint[0] = clipped2[bestIdx].v;
        manifold.contactPenetration[0] = std::max(-minSep, 0.0f);
        manifold.contactCount = 1;
    }

    if (manifold.contactCount > 0)
    {
        manifold.penetration = manifold.contactPenetration[0];
        if (manifold.contactCount == 2)
            manifold.penetration = std::max(manifold.contactPenetration[0], manifold.contactPenetration[1]);
    }

    return manifold;
}
Manifold OBBvsCircleManifold(const RigidBody2D &bodyA, const Box &boxA, const RigidBody2D &bodyB, const Circle &circleB)
{
    Manifold manifold;
    manifold.colliding = false;
    (void)bodyB;

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
        manifold.contactPenetration[0] = manifold.penetration;
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
        manifold.contactPenetration[0] = manifold.penetration;
        manifold.contactCount = 1;
    }
    return manifold;
}
Manifold GetManifold(RigidBody2D *a, RigidBody2D *b)
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
