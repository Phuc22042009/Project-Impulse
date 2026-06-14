#pragma once
#include "../physics/RigidBody2D.h"
#include "Collision.h"
#include "../math/Vec2.h"
#include <vector>
#include <memory>
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

        float accumulatedPseudoImpulse = 0.0f;
        float pseudoBias = 0.0f;

        float bias;
    };
inline void WarmStart(ContactConstraint& constraint)
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
inline void MatchAndWarmStart(std::vector<ContactConstraint>& constraints, const std::vector<ContactConstraint>& previousConstraints)
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
                    if ((constraint.contactPoint - prev.contactPoint).LenSq() < matchDistanceSq)
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
    }  
inline ContactConstraint GenerateContactConstraint(const Manifold& manifold, int pointIndex, RigidBody2D& a, RigidBody2D& b, float dt)
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
        constraint.accumulatedPseudoImpulse = 0;
        
        Vec2 vRel = (a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x)) - (b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x));
        float vRelDotN = DotProduct(vRel, constraint.normal);
        float restitution = std::min(a.restitution, b.restitution);

        constraint.bias = (vRelDotN < -1.0f) ? -restitution * vRelDotN : 0.0f;
        
        float slop = 0.05f;
        float biasFactor = 0.2f;
        constraint.pseudoBias = (biasFactor / dt) * std::max(constraint.penetration - slop, 0.0f);

        return constraint;
    } 
inline void ResolveContactConstraint(ContactConstraint& constraint, float dt)
    {
        //Normal impulse
        RigidBody2D& a = *constraint.bodyA;
        RigidBody2D& b = *constraint.bodyB;

        Vec2 vContactA = a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x);
        Vec2 vContactB = b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x);
        Vec2 vRel = vContactA - vContactB;

        float vRelDotN = DotProduct(vRel, constraint.normal);
        float rnA = CrossProduct(constraint.rA, constraint.normal);
        float rnB = CrossProduct(constraint.rB, constraint.normal);

        float denominator = a.invMass + b.invMass + rnA * a.invInertia * rnA + rnB * b.invInertia * rnB;
        if (denominator > 1e-8f) 
        {
            float lambda = (-vRelDotN + constraint.bias) / denominator;
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
        float rtA = CrossProduct(constraint.rA, constraint.tangent);
        float rtB = CrossProduct(constraint.rB, constraint.tangent);
        float frictionDenominator = a.invMass + b.invMass + rtA * a.invInertia * rtA + rtB * b.invInertia * rtB;
        if (frictionDenominator > 1e-8f) 
        {
            Vec2 vContactA2 = a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x);
            Vec2 vContactB2 = b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x);
            Vec2 vRel2 = vContactA2 - vContactB2;
            float vRelDotTangent = DotProduct(vRel2, constraint.tangent);
            
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

        Vec2 pseudoContactA = a.pseudoVelocity + Vec2(-a.pseudoAngularVelocity * constraint.rA.y, a.pseudoAngularVelocity * constraint.rA.x);
        Vec2 pseudoContactB = b.pseudoVelocity + Vec2(-b.pseudoAngularVelocity * constraint.rB.y, b.pseudoAngularVelocity * constraint.rB.x);   
        Vec2 pseudoVRel = pseudoContactA - pseudoContactB;
        float pseudoVRelDotN = DotProduct(pseudoVRel, constraint.normal);

        float pseudoLambda = (-pseudoVRelDotN + constraint.pseudoBias) / denominator;

        float oldAccumulatedPseudoImpulse = constraint.accumulatedPseudoImpulse;
        constraint.accumulatedPseudoImpulse = std::max(oldAccumulatedPseudoImpulse + pseudoLambda, 0.0f);
        float actualPseudoLambda = constraint.accumulatedPseudoImpulse - oldAccumulatedPseudoImpulse;

        Vec2 pseudoImpulse = constraint.normal * actualPseudoLambda;

        a.pseudoVelocity = a.pseudoVelocity + pseudoImpulse * a.invMass;
        a.pseudoAngularVelocity += CrossProduct(constraint.rA, pseudoImpulse) * a.invInertia;
        b.pseudoVelocity = b.pseudoVelocity - pseudoImpulse * b.invMass;
        b.pseudoAngularVelocity -= CrossProduct(constraint.rB, pseudoImpulse) * b.invInertia;
    }
inline void SolveConstraints(std::vector<ContactConstraint>& constraints, const std::vector<ContactConstraint>& previousConstraints, float dt)
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