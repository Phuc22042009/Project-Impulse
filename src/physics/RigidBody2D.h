    #pragma once
    #include "../math/Vec2.h"
    #include "Shape.h"
    struct RigidBody2D
    {
        Shape* shape;

        Vec2 position;
        Vec2 force;
        Vec2 velocity;
        
        float torque;
        float angularVelocity;
        float angle;

        float mass;
        float invMass;
        float inertia;
        float invInertia;
        float restitution;
        float staticFriction;

        bool isAwake = true;
        float sleepTimer = 0;
        
        RigidBody2D(float mass, Shape* shape): mass(mass), invMass(0), position(), force(), velocity(), torque(0), shape(shape), restitution(0.0f), staticFriction(0.5f), angularVelocity(0), angle(0)
        {
            invMass = (mass == 0) ? 0 : 1 / mass;
            inertia = shape->ComputeInertia(mass);
            invInertia = (inertia == 0) ? 0 : 1 / inertia;
        }

        void ApplyForce(Vec2 force, Vec2 impactPoint)
        {
            isAwake = true;
            sleepTimer = 0;

            Vec2 centerToForce = impactPoint - position;
            torque += CrossProduct(centerToForce, force);
            this->force = this->force + force;
        }
        void Integrate(float dt)
        {
            if(!isAwake) return;

            Vec2 acceleration = force * invMass;
            velocity = velocity + acceleration * dt;

            position = position + velocity * dt;   
            float angularAcceleration = torque * invInertia;
            angularVelocity += angularAcceleration * dt;
            angle += angularVelocity * dt;
            force = Vec2();
            torque = 0;
        }
    };
void UpdateSleep(RigidBody2D& body, float dt)
{

    if (body.invMass == 0) return;
    if (!body.isAwake) return;

    float velocitySq = body.velocity.LenSq();
    float angularVelocitySq = body.angularVelocity * body.angularVelocity;

    const float linearSleepThresholdSq = 0.01f * 0.01f;
    const float angularSleepThresholdSq = 0.01f * 0.01f;

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