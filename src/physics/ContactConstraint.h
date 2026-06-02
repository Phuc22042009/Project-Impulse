#pragma once
#include "../physics/RigidBody2D.h"
#include "Collision.h"
#include "../math/Vec2.h"
#include <vector>
struct ContactConstraint
    {
        RigidBody2D* bodyA;
        RigidBody2D* bodyB;
        Vec2 rA;
        Vec2 rB;
        Vec2 normal;
        Vec2 contactPoint;
        Vec2 tangent;
        float penetration;
        float accumulatedNormalImpulse;
        float accumulatedFrictionImpulse;
        float bias;
    };
void WarmStart(ContactConstraint& constraint)
    {
        RigidBody2D& a = *constraint.bodyA;
        RigidBody2D& b = *constraint.bodyB;

        constraint.rA = constraint.contactPoint - a.position;
        constraint.rB = constraint.contactPoint - b.position;

        Vec2 normalImpulse = constraint.normal * constraint.accumulatedNormalImpulse;
        Vec2 frictionImpulse = constraint.tangent * constraint.accumulatedFrictionImpulse;
        Vec2 totalImpulse = normalImpulse + frictionImpulse;

        a.velocity = a.velocity + totalImpulse * a.invMass;
        a.angularVelocity += CrossProduct(constraint.rA, totalImpulse) * a.invInertia;
        b.velocity = b.velocity - totalImpulse * b.invMass;
        b.angularVelocity -= CrossProduct(constraint.rB, totalImpulse) * b.invInertia;
    }
void MatchAndWarmStart(std::vector<ContactConstraint>& constraints, const std::vector<ContactConstraint>& previousConstraints)
    {
        const float matchDistanceSq = 4.0f * 4.0f;
        for (ContactConstraint& constraint : constraints) 
        {
            for (const ContactConstraint& prev : previousConstraints) 
            {
                if (constraint.bodyA == prev.bodyA && constraint.bodyB == prev.bodyB)
                {
                    if ((constraint.contactPoint - prev.contactPoint).LenSq() < matchDistanceSq)
                    {
                        constraint.accumulatedNormalImpulse = prev.accumulatedNormalImpulse;
                        float tangentDot = DotProduct(constraint.tangent, prev.tangent);
                        if (tangentDot > 0.0f)
                            constraint.accumulatedFrictionImpulse = prev.accumulatedFrictionImpulse;
                        else
                            constraint.accumulatedFrictionImpulse = -prev.accumulatedFrictionImpulse;
                        WarmStart(constraint);
                        break;
                    }
                }
                else if (constraint.bodyA == prev.bodyB && constraint.bodyB == prev.bodyA)
                {
                    constraint.accumulatedNormalImpulse = prev.accumulatedNormalImpulse;
                    float tangentDot = DotProduct(constraint.tangent, prev.tangent);
                    if (tangentDot > 0.0f)
                        constraint.accumulatedFrictionImpulse = -prev.accumulatedFrictionImpulse;
                    else
                        constraint.accumulatedFrictionImpulse = prev.accumulatedFrictionImpulse;
                    WarmStart(constraint);
                    break;
                }
            }
        }
    }  
ContactConstraint GenerateContactConstraint(const Manifold& manifold, int pointIndex, RigidBody2D& a, RigidBody2D& b)
    {
        if (!a.isAwake) {a.isAwake = true, a.sleepTimer = 0;}
        if (!b.isAwake) {b.isAwake = true, b.sleepTimer = 0;}

        ContactConstraint constraint;
        constraint.bodyA = &a;
        constraint.bodyB = &b;
        constraint.normal = manifold.normal;
        constraint.tangent = Vec2(-manifold.normal.y, manifold.normal.x);
        constraint.contactPoint = manifold.contactPoint[pointIndex];
        constraint.penetration = manifold.penetration;
        constraint.rA = constraint.contactPoint - a.position;
        constraint.rB = constraint.contactPoint - b.position;
        constraint.accumulatedNormalImpulse = 0;
        constraint.accumulatedFrictionImpulse = 0;
        constraint.bias = 0;

        return constraint;
    } 
void ResolveContactConstraint(ContactConstraint& constraint, float dt)
    {
        //Normal impulse
        RigidBody2D& a = *constraint.bodyA;
        RigidBody2D& b = *constraint.bodyB;

        Vec2 vContactA = a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x);
        Vec2 vContactB = b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x);
        Vec2 vRel = vContactA - vContactB;

        float vRelDotN = DotProduct(vRel, constraint.normal);
        float denominator = a.invMass + b.invMass + CrossProduct(constraint.rA, constraint.normal) * a.invInertia * CrossProduct(constraint.rA, constraint.normal) + CrossProduct(constraint.rB, constraint.normal) * b.invInertia * CrossProduct(constraint.rB, constraint.normal);
        if (denominator > 1e-8f) 
        {
            float slop = 0.3f;
            float biasFactor = 0.4f;
            constraint.bias = (biasFactor / dt) * std::max(constraint.penetration - slop, 0.0f);

            float restitution = std::min(a.restitution, b.restitution);

            float lambda = (-(1.0f + restitution) * vRelDotN + constraint.bias) / denominator;
            float oldAccumulatedNormalImpulse = constraint.accumulatedNormalImpulse;
            constraint.accumulatedNormalImpulse = std::max(oldAccumulatedNormalImpulse + lambda, 0.0f);
            float actualLambda = constraint.accumulatedNormalImpulse - oldAccumulatedNormalImpulse;

            Vec2 impulse = constraint.normal * actualLambda;
            a.velocity = a.velocity + impulse * a.invMass;
            a.angularVelocity += CrossProduct(constraint.rA, impulse) * a.invInertia;
            b.velocity = b.velocity - impulse * b.invMass;
            b.angularVelocity -= CrossProduct(constraint.rB, impulse) * b.invInertia;
        }

        //Friction impulse
        Vec2 vContactA2 = a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x);
        Vec2 vContactB2 = b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x);
        Vec2 vRel2 = vContactA2 - vContactB2;

        float vRelDotTangent = DotProduct(vRel2, constraint.tangent);
        float frictionDenominator = a.invMass + b.invMass + CrossProduct(constraint.rA, constraint.tangent) * a.invInertia * CrossProduct(constraint.rA, constraint.tangent) + CrossProduct(constraint.rB, constraint.tangent) * b.invInertia * CrossProduct(constraint.rB, constraint.tangent);
        if (frictionDenominator > 1e-8f) 
        {
            float jt = -vRelDotTangent / frictionDenominator;
            float mu = (a.staticFriction + b.staticFriction) * 0.5f;
            float maxFriction = mu * constraint.accumulatedNormalImpulse;

            float oldAccumulatedFrictionImpulse = constraint.accumulatedFrictionImpulse;
            constraint.accumulatedFrictionImpulse = std::clamp(oldAccumulatedFrictionImpulse + jt, -maxFriction, maxFriction);
            float actualFrictionLambda = constraint.accumulatedFrictionImpulse - oldAccumulatedFrictionImpulse;

            Vec2 frictionImpulse = constraint.tangent * actualFrictionLambda;

            a.velocity = a.velocity + frictionImpulse * a.invMass;
            a.angularVelocity += CrossProduct(constraint.rA, frictionImpulse) * a.invInertia;
            b.velocity = b.velocity - frictionImpulse * b.invMass;
            b.angularVelocity -= CrossProduct(constraint.rB, frictionImpulse) * b.invInertia;
        }
    }
void SolveConstraints(std::vector<ContactConstraint>& constraints, const std::vector<ContactConstraint>& previousConstraints, float dt)
    {
        const int iterations = 10;
        MatchAndWarmStart(constraints, previousConstraints);
        for (int i = 0; i < iterations; ++i) 
        {
            for (ContactConstraint& constraint : constraints) 
            {
                ResolveContactConstraint(constraint, dt);
            }
        }
    }