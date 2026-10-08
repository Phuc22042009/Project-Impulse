#include "RigidBody2D.h"
#include <cassert>
#include <iostream>
#include <cmath>

RigidBody2D::RigidBody2D(float mass, std::unique_ptr<Shape> shape) : shape(std::move(shape)), position(), force(), velocity(), pseudoVelocity(), pseudoAngularVelocity(0), angularVelocity(0), angle(0), mass(mass), invMass(0), torque(0), inertia(0.0f), invInertia(0.0f), restitution(0.0f), staticFriction(0.5f)
{
    invMass = (mass == 0) ? 0 : 1 / mass;
    inertia = this->shape->ComputeInertia(mass);
    invInertia = (inertia == 0) ? 0 : 1 / inertia;
    if (invMass == 0.0f)
    {
        isAwake = false;
    }
}

void RigidBody2D::ApplyForce(const Vec2 &force, const Vec2 &impactPoint)
{
    Vec2 centerToForce = impactPoint - position;
    torque += CrossProduct(centerToForce, force);
    this->force = this->force + force;
    WakeUp();
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

    velocity = velocity * (1.0f / (1.0f + 0.1f * dt));
    angularVelocity *= 1.0f / (1.0f + 0.5f * dt);

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

void RigidBody2D::UpdateSleep(float dt)
{
    if (invMass == 0) return;
    if (!isAwake) return;

    float velocitySq = velocity.LenSq();
    float angularSq  = angularVelocity * angularVelocity;

    const float linearSleepThresholdSq  = 6.0f * 6.0f;
    const float angularSleepThresholdSq = 0.1f * 0.1f;

    if (velocitySq < linearSleepThresholdSq && angularSq < angularSleepThresholdSq)
    {
        sleepTimer += dt;

        if (sleepTimer > 0.1f)
        {
            if (velocitySq < 4.0f) velocity = Vec2();
            if (angularSq < 0.05f * 0.05f) angularVelocity = 0;
        }

        if (sleepTimer >= 0.5f)
        {
            isAwake               = false;
            velocity              = Vec2();
            angularVelocity       = 0;
            pseudoVelocity        = Vec2();
            pseudoAngularVelocity = 0;
        }
    }
    else
    {
        sleepTimer = std::max(sleepTimer - dt * 2.0f, 0.0f);
    }
}
void RigidBody2D::WakeUp()
{
    if (this->invMass == 0) return;
    if (!this->isAwake)
    {
        this->isAwake = true;
        this->sleepTimer = 0.0f;
    }
}