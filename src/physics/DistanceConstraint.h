#pragma once
#include "RigidBody2D.h"
#include <vector>
#include <memory>
struct DistanceConstraint
{
    int bodyAIndex;
    int bodyBIndex;

    float restLength;
    float accumulatedImpulse;
};
inline void SolveDistanceConstraint(DistanceConstraint &constraint, std::vector<RigidBody2D> &bodies, float dt)
{
    RigidBody2D &bodyA = bodies[constraint.bodyAIndex];
    RigidBody2D &bodyB = bodies[constraint.bodyBIndex];

    Vec2 d = bodyB.position - bodyA.position;
    float currentLength = d.Len();
    Vec2 dHat = (currentLength > 1e-8f) ? d / currentLength : Vec2(0, 0);
    float constraintError = currentLength - constraint.restLength;

    float Cdot = DotProduct(dHat, bodyA.velocity) - DotProduct(dHat, bodyB.velocity);
    float denominator = bodyA.invMass + bodyB.invMass;

    float bias = (0.2f / dt) * constraintError;
    if (denominator < 1e-8f)
        return;
    float lambda = (-Cdot + bias) / denominator;

    Vec2 impulse = dHat * lambda;
    bodyA.velocity = bodyA.velocity + impulse * bodyA.invMass;
    bodyB.velocity = bodyB.velocity - impulse * bodyB.invMass;
}