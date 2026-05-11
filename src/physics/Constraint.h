#include "../physics/RigidBody2D.h"
#include "Collision.h"
#include "../math/Vec2.h"

struct ContactConstraint
    {
        RigidBody2D* bodyA;
        RigidBody2D* bodyB;
        Vec2 rA;
        Vec2 rB;
        Vec2 normal;
        Vec2 contactPoint;
        float penetration;
        float accumulatedNormalImpulse;
        float accumulatedFrictionImpulse;
    };
void ResolveContactConstraint(ContactConstraint& constraint, float dt)
    {
        //Normal impulse
        RigidBody2D& a = *constraint.bodyA;
        RigidBody2D& b = *constraint.bodyB;

        Vec2 vContactA = a.velocity + Vec2(-a.angularVelocity * constraint.rA.y, a.angularVelocity * constraint.rA.x);
        Vec2 vContactB = b.velocity + Vec2(-b.angularVelocity * constraint.rB.y, b.angularVelocity * constraint.rB.x);
        Vec2 vRel = vContactA - vContactB;

        float vRelDotN = DotProduct(vRel, constraint.normal);
        if (vRelDotN >= 0) return;
        float denominator = a.invMass + b.invMass + CrossProduct(constraint.rA, constraint.normal) * a.invInertia * CrossProduct(constraint.rA, constraint.normal) + CrossProduct(constraint.rB, constraint.normal) * b.invInertia * CrossProduct(constraint.rB, constraint.normal);
        if (denominator < 1e-8f) return;

        float bias = (0.2 / dt) * std::max(constraint.penetration - 0.01f, 0.0f);
        float lambda = (-vRelDotN + bias) / denominator;
        float oldAccumulatedNormalImpulse = constraint.accumulatedNormalImpulse;
        constraint.accumulatedNormalImpulse = std::max(oldAccumulatedNormalImpulse + lambda, 0.0f);

        float actualLambda = constraint.accumulatedNormalImpulse - oldAccumulatedNormalImpulse;
        Vec2 impulse = constraint.normal * actualLambda;

        a.velocity = a.velocity + impulse * a.invMass;
        a.angularVelocity += CrossProduct(constraint.rA, impulse) * a.invInertia;
        b.velocity = b.velocity - impulse * b.invMass;
        b.angularVelocity -= CrossProduct(constraint.rB, impulse) * b.invInertia;


        //Friction impulse

        Vec2 tangent = vRel - constraint.normal * vRelDotN;
        if (tangent.Len() < 1e-8f) return;
        Vec2 tangentDir = tangent / tangent.Len();
        float frictionDenom = a.invMass + b.invMass + CrossProduct(constraint.rA, tangentDir) * a. invInertia * CrossProduct(constraint.rB, tangentDir) * b.invInertia;
        float jt = -DotProduct(vRel, tangentDir) / frictionDenom;

        float mu = (a.staticFriction + b.staticFriction) * 0.5f;
        float maxFriction = mu * constraint.accumulatedNormalImpulse;
        float oldAccumulatedFrictionImpulse = constraint.accumulatedFrictionImpulse;
        constraint.accumulatedFrictionImpulse = std::clamp(oldAccumulatedNormalImpulse + jt, -maxFriction, maxFriction);
        float actualFrictionLambda = constraint.accumulatedFrictionImpulse - oldAccumulatedFrictionImpulse;

        Vec2 frictionImpulse = tangentDir * actualFrictionLambda;
        a.velocity = a.velocity + frictionImpulse * a.invMass;
        a.angularVelocity += CrossProduct(constraint.rA, frictionImpulse) * a.invInertia;
        b.velocity = b.velocity - frictionImpulse * b.invMass;
        b.angularVelocity -= CrossProduct(constraint.rB, frictionImpulse) * b.invInertia;
    }