#pragma once
#include "Shape.h"
#include "../math/Vec2.h"
#include "../physics/RigidBody2D.h"
#include <array>
struct Manifold
{
    bool colliding = false;
    Vec2 normal;
    float penetration = 0;
    Vec2 contactPoint[2];
    float contactPenetration[2] = {0.0f, 0.0f};
    int contactCount = 0;
};

Manifold GetCircleManifold(const Circle &a, const Circle &b);
Manifold AABBvsCircleManifold(const AABB &box, const Circle &circle);
void GetVertices(const RigidBody2D &body, const Box &box, Vec2 vertices[4]);
void ProjectOBB(const Vec2 vertices[4], const Vec2 &axis, float &minVal, float &maxVal);
float TestAxis(const Vec2 verticesA[4], const Vec2 verticesB[4], const Vec2 &axis);
Manifold OBBvsOBBManifold(const RigidBody2D &bodyA, const Box &boxA, const RigidBody2D &bodyB, const Box &boxB);
Manifold OBBvsCircleManifold(const RigidBody2D &bodyA, const Box &boxA, const RigidBody2D &bodyB, const Circle &circleB);
Manifold GetManifold(RigidBody2D *a, RigidBody2D *b);