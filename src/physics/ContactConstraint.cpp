#include "ContactConstraint.h"
#include "../math/MathUtils.h"
#include <algorithm>
#include <cmath>

void WarmStart(ContactConstraint &constraint, std::vector<RigidBody2D> &bodies)
{
    RigidBody2D &a = bodies[constraint.bodyAIndex];
    RigidBody2D &b = bodies[constraint.bodyBIndex];

    constraint.rA = constraint.contactPoint - a.position;
    constraint.rB = constraint.contactPoint - b.position;

    Vec2 normalImpulse = constraint.normal * constraint.accumulatedNormalImpulse;
    Vec2 frictionImpulse = constraint.tangent * constraint.accumulatedFrictionImpulse;
    Vec2 totalImpulse = normalImpulse + frictionImpulse;

    if (a.isAwake)
    {
        a.velocity = a.velocity + totalImpulse * a.invMass;
        a.angularVelocity += CrossProduct(constraint.rA, totalImpulse) * a.invInertia;
    }
    if (b.isAwake)
    {
        b.velocity = b.velocity - totalImpulse * b.invMass;
        b.angularVelocity -= CrossProduct(constraint.rB, totalImpulse) * b.invInertia;
    }
}
void MatchAndWarmStart(std::vector<ContactConstraint> &constraints, const std::vector<ContactConstraint> &previousConstraints, std::vector<RigidBody2D> &bodies)
{
    const float matchDistanceSq = 4.0f * 4.0f;
    for (ContactConstraint &constraint : constraints)
    {
        for (const ContactConstraint &prev : previousConstraints)
        {
            if (constraint.bodyAIndex == prev.bodyAIndex && constraint.bodyBIndex == prev.bodyBIndex)
            {
                if ((constraint.contactPoint - prev.contactPoint).LenSq() < matchDistanceSq)
                {
                    constraint.accumulatedNormalImpulse = prev.accumulatedNormalImpulse;
                    float tangentDot = DotProduct(constraint.tangent, prev.tangent);
                    if (tangentDot > 0.0f)
                        constraint.accumulatedFrictionImpulse = prev.accumulatedFrictionImpulse;
                    else
                        constraint.accumulatedFrictionImpulse = -prev.accumulatedFrictionImpulse;
                    WarmStart(constraint, bodies);
                    break;
                }
            }
            else if (constraint.bodyAIndex == prev.bodyBIndex && constraint.bodyBIndex == prev.bodyAIndex)
            {
                if ((constraint.contactPoint - prev.contactPoint).LenSq() < matchDistanceSq)
                {
                    constraint.accumulatedNormalImpulse = prev.accumulatedNormalImpulse;
                    float tangentDot = DotProduct(constraint.tangent, prev.tangent);
                    if (tangentDot > 0.0f)
                        constraint.accumulatedFrictionImpulse = -prev.accumulatedFrictionImpulse;
                    else
                        constraint.accumulatedFrictionImpulse = prev.accumulatedFrictionImpulse;
                    WarmStart(constraint, bodies);
                    break;
                }
            }
        }
    }
}
ContactConstraint GenerateContactConstraint(const Manifold &manifold, std::vector<RigidBody2D> &bodies, int pointIndex, int indexA, int indexB, float dt)
{
    RigidBody2D &a = bodies[indexA];
    RigidBody2D &b = bodies[indexB];

    ContactConstraint constraint;
    constraint.bodyAIndex = indexA;
    constraint.bodyBIndex = indexB;
    constraint.normal = manifold.normal;
    constraint.tangent = Vec2(-manifold.normal.y, manifold.normal.x);
    constraint.contactPoint = manifold.contactPoint[pointIndex];
    constraint.penetration = manifold.contactPenetration[pointIndex];
    constraint.rA = constraint.contactPoint - a.position;
    constraint.rB = constraint.contactPoint - b.position;
    constraint.accumulatedNormalImpulse = 0;
    constraint.accumulatedFrictionImpulse = 0;
    constraint.accumulatedPseudoImpulse = 0;

    Vec2 vRel = (a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x)) - (b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x));
    float vRelDotN = DotProduct(vRel, constraint.normal);
    float restitution = std::min(a.restitution, b.restitution);

    const float restitutionVelocityThreshold = 30.0f;
    constraint.bias = (vRelDotN < -restitutionVelocityThreshold) ? -restitution * vRelDotN : 0.0f;

    float slop = 0.5f;
    float biasFactor = 0.2f;
    constraint.pseudoBias = (biasFactor / dt) * std::max(constraint.penetration - slop, 0.0f);

    return constraint;
}
void ResolveContactConstraint(ContactConstraint &constraint, std::vector<RigidBody2D> &bodies, float dt)
{
    (void)dt;
    RigidBody2D &a = bodies[constraint.bodyAIndex];
    RigidBody2D &b = bodies[constraint.bodyBIndex];

    constraint.rA = constraint.contactPoint - a.position;
    constraint.rB = constraint.contactPoint - b.position;

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
        if (a.isAwake)
        {
            a.velocity = a.velocity + impulse * a.invMass;
            a.angularVelocity += CrossProduct(constraint.rA, impulse) * a.invInertia;
        }
        if (b.isAwake)
        {
            b.velocity = b.velocity - impulse * b.invMass;
            b.angularVelocity -= CrossProduct(constraint.rB, impulse) * b.invInertia;
        }
    }

    Vec2 vContactA2 = a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x);
    Vec2 vContactB2 = b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x);
    Vec2 vRel2 = vContactA2 - vContactB2;
    float vRelDotTangent = DotProduct(vRel2, constraint.tangent);

    float rtA = CrossProduct(constraint.rA, constraint.tangent);
    float rtB = CrossProduct(constraint.rB, constraint.tangent);
    float frictionDenominator = a.invMass + b.invMass + rtA * a.invInertia * rtA + rtB * b.invInertia * rtB;
    if (frictionDenominator > 1e-8f)
    {
        float jt = -vRelDotTangent / frictionDenominator;
        float mu = std::sqrt(a.staticFriction * b.staticFriction);
        float maxFriction = mu * constraint.accumulatedNormalImpulse;

        float oldAccumulatedFrictionImpulse = constraint.accumulatedFrictionImpulse;
        constraint.accumulatedFrictionImpulse = std::clamp(oldAccumulatedFrictionImpulse + jt, -maxFriction, maxFriction);
        float actualFrictionLambda = constraint.accumulatedFrictionImpulse - oldAccumulatedFrictionImpulse;
        Vec2 frictionImpulse = constraint.tangent * actualFrictionLambda;

        if (a.isAwake)
        {
            a.velocity = a.velocity + frictionImpulse * a.invMass;
            a.angularVelocity += CrossProduct(constraint.rA, frictionImpulse) * a.invInertia;
        }
        if (b.isAwake)
        {           
            b.velocity = b.velocity - frictionImpulse * b.invMass;
            b.angularVelocity -= CrossProduct(constraint.rB, frictionImpulse) * b.invInertia;
        }
    }

    Vec2 pseudoContactA = a.pseudoVelocity + Vec2(-a.pseudoAngularVelocity * constraint.rA.y, a.pseudoAngularVelocity * constraint.rA.x);
    Vec2 pseudoContactB = b.pseudoVelocity + Vec2(-b.pseudoAngularVelocity * constraint.rB.y, b.pseudoAngularVelocity * constraint.rB.x);
    Vec2 pseudoVRel = pseudoContactA - pseudoContactB;
    float pseudoVRelDotN = DotProduct(pseudoVRel, constraint.normal);
    if (denominator > 1e-8f)
    {
        float pseudoLambda = (-pseudoVRelDotN + constraint.pseudoBias) / denominator;

        float oldAccumulatedPseudoImpulse = constraint.accumulatedPseudoImpulse;
        constraint.accumulatedPseudoImpulse = std::max(oldAccumulatedPseudoImpulse + pseudoLambda, 0.0f);
        float actualPseudoLambda = constraint.accumulatedPseudoImpulse - oldAccumulatedPseudoImpulse;

        Vec2 pseudoImpulse = constraint.normal * actualPseudoLambda;
        
        if (a.isAwake)
        {
            a.pseudoVelocity = a.pseudoVelocity + pseudoImpulse * a.invMass;
            a.pseudoAngularVelocity += CrossProduct(constraint.rA, pseudoImpulse) * a.invInertia;
        }
        if (b.isAwake)
        {
            b.pseudoVelocity = b.pseudoVelocity - pseudoImpulse * b.invMass;
            b.pseudoAngularVelocity -= CrossProduct(constraint.rB, pseudoImpulse) * b.invInertia;
        }
    }
}
void SolveContactConstraints(std::vector<ContactConstraint> &constraints, const std::vector<ContactConstraint> &previousConstraints, std::vector<RigidBody2D> &bodies, float dt)
{
    const int iterations = 10;
    MatchAndWarmStart(constraints, previousConstraints, bodies);
    for (int i = 0; i < iterations; ++i)
    {
        for (ContactConstraint &constraint : constraints)
        {
            ResolveContactConstraint(constraint, bodies, dt);
        }
    }
}
