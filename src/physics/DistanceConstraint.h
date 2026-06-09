#pragma once
#include "RigidBody2D.h"

struct DistanceConstraint
{
    RigidBody2D* bodyA;
    RigidBody2D* bodyB;

    float restLength;
    float accumulatedImpulse;
};
inline void SolveDistanceConstraint(DistanceConstraint& constraint, float dt)
{
    Vec2 d = constraint.bodyB->position - constraint.bodyA->position;
    float currentLength = d.Len();
    Vec2 dHat = (currentLength > 1e-8f) ? d / currentLength : Vec2(0, 0);
    float constraintError = currentLength - constraint.restLength;
    
    float Cdot = DotProduct(dHat, constraint.bodyA->velocity) - DotProduct(dHat, constraint.bodyB->velocity);
    float denominator = constraint.bodyA->invMass + constraint.bodyB->invMass;

    float bias = (0.2f / dt) * constraintError;
    float lambda = (-Cdot + bias) / denominator;

    Vec2 impulse = dHat * lambda;
    constraint.bodyA->velocity = constraint.bodyA->velocity + impulse * constraint.bodyA->invMass;
    constraint.bodyB->velocity = constraint.bodyB->velocity - impulse * constraint.bodyB->invMass;
}