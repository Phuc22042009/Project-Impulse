#include "DistanceConstraint.h"
#include "../math/MathUtils.h"
#include <cmath>

void WarmStartDistanceConstraint(DistanceConstraint &constraint, std::vector<RigidBody2D> &bodies)
{
    RigidBody2D &bodyA = bodies[constraint.bodyAIndex];
    RigidBody2D &bodyB = bodies[constraint.bodyBIndex];

    Vec2 d = bodyB.position - bodyA.position;
    float currentLength = d.Len();
    if (currentLength < 1e-8f) return;

    Vec2 dHat = d / currentLength;
    Vec2 impulse = dHat * constraint.accumulatedImpulse;

    bodyA.velocity = bodyA.velocity + impulse * bodyA.invMass;
    bodyB.velocity = bodyB.velocity - impulse * bodyB.invMass;
}

void SolveDistanceConstraint(DistanceConstraint &constraint, std::vector<RigidBody2D> &bodies, float dt)
{
    RigidBody2D &bodyA = bodies[constraint.bodyAIndex];
    RigidBody2D &bodyB = bodies[constraint.bodyBIndex];

    Vec2 d = bodyB.position - bodyA.position;
    float currentLength = d.Len();
    Vec2 dHat = (currentLength > 1e-8f) ? d / currentLength : Vec2(0, 0);

    float constraintError = currentLength - constraint.restLength;

    float Cdot = DotProduct(dHat, bodyB.velocity - bodyA.velocity);
    float denominator = bodyA.invMass + bodyB.invMass;
    
    if (denominator < 1e-8f)
        return;

    float biasFactor = 0.2f;
    float bias = (biasFactor / dt) * constraintError;

    float lambda = (Cdot + bias) / denominator;

    constraint.accumulatedImpulse += lambda;

    Vec2 impulse = dHat * lambda;
    bodyA.velocity = bodyA.velocity + impulse * bodyA.invMass;
    bodyB.velocity = bodyB.velocity - impulse * bodyB.invMass;
}