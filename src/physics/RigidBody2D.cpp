#include "RigidBody2D.h"
#include <cassert>
#include <cmath>

RigidBody2D::RigidBody2D(float mass, std::unique_ptr<Shape> shape) : shape(std::move(shape)), position(), force(), velocity(), pseudoVelocity(), pseudoAngularVelocity(0), angularVelocity(0), angle(0), mass(mass), invMass(0), torque(0), inertia(0.0f), invInertia(0.0f), restitution(0.0f), staticFriction(0.5f)
{
    invMass = (mass == 0) ? 0 : 1 / mass;
    inertia = this->shape->ComputeInertia(mass);
    invInertia = (inertia == 0) ? 0 : 1 / inertia;
}

void RigidBody2D::ApplyForce(const Vec2 &force, const Vec2 &impactPoint)
{
    isAwake = true;
    sleepTimer = 0;

    Vec2 centerToForce = impactPoint - position;
    torque += CrossProduct(centerToForce, force);
    this->force = this->force + force;
}
void RigidBody2D::IntegrateVelocities(float dt)
{
    if (!isAwake)
        return;
    if (invMass == 0)
        return;

    Vec2 acceleration = force * invMass;
    velocity = velocity + acceleration * dt;

    float angularAcceleration = torque * invInertia;
    angularVelocity += angularAcceleration * dt;

    velocity = velocity * std::pow(0.98f, dt);
    angularVelocity *= std::pow(0.98f, dt);

    force = Vec2();
    torque = 0;
}

void RigidBody2D::IntegratePositions(float dt)
{
    if (!isAwake)
        return;

    position = position + (velocity + pseudoVelocity) * dt;
    angle += (angularVelocity + pseudoAngularVelocity) * dt;

    pseudoVelocity = Vec2();
    pseudoAngularVelocity = 0;
}

void RigidBody2D::UpdateSleep(RigidBody2D &body, float dt)
{

    if (body.invMass == 0)
        return;
    if (!body.isAwake)
        return;

    float velocitySq = body.velocity.LenSq();
    float angularVelocitySq = body.angularVelocity * body.angularVelocity;

    const float linearSleepThresholdSq = 2.0f * 2.0f;
    const float angularSleepThresholdSq = 0.02f * 0.02f;

    if (velocitySq < linearSleepThresholdSq && angularVelocitySq < angularSleepThresholdSq)
    {
        body.sleepTimer += dt;
        if (body.sleepTimer >= 0.5f)
        {
            body.isAwake = false;
            body.velocity = Vec2();
            body.angularVelocity = 0;
        }
    }
    else
    {
        body.sleepTimer = 0;
        body.isAwake = true;
    }
}