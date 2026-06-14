#pragma once
#include "RigidBody2D.h"
#include <memory>
struct DistanceConstraint
{
    RigidBody2D* bodyA;
    RigidBody2D* bodyB;

    float restLength;
    float accumulatedImpulse;
};
inline void ReSolveDistanceConstraint(DistanceConstraint& constraint, float dt)
{
    RigidBody2D* bodyA = constraint.bodyA;
    RigidBody2D* bodyB = constraint.bodyB;

    if (!bodyA || !bodyB) return;

    Vec2 d = bodyB->position - bodyA->position;
    float currentLength = d.Len();
    Vec2 dHat = (currentLength > 1e-8f) ? d / currentLength : Vec2(0, 0);
    float constraintError = currentLength - constraint.restLength;
    
    float Cdot = DotProduct(dHat, bodyA->velocity) - DotProduct(dHat, bodyB->velocity);
    float denominator = bodyA->invMass + bodyB->invMass;

    float bias = (0.2f / dt) * constraintError;
    if (denominator < 1e-8f) return;
    float lambda = (-Cdot + bias) / denominator;

    Vec2 impulse = dHat * lambda;
    bodyA->velocity = bodyA->velocity + impulse * bodyA->invMass;
    bodyB->velocity = bodyB->velocity - impulse * bodyB->invMass;
}
inline void SolveDistanceConstraint(DistanceConstraint& constraint, float dt)
{
    for (int i = 0; i < 10; i++) 
    {
        ReSolveDistanceConstraint(constraint, dt);
    }
}