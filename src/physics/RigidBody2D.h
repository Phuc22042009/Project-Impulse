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
        RigidBody2D(float mass, Shape* shape): mass(mass), invMass(0), position(), force(), velocity(), torque(0), shape(shape)
        {
            invMass = (mass == 0) ? 0 : 1/mass;
            inertia = shape->ComputeInertia(mass);
            invInertia = (inertia == 0) ? 0 : 1/inertia;
        }

        void ApplyForce(Vec2 force, Vec2 impactPoint)
        {
            Vec2 centerToForce = impactPoint - position;
            torque += CrossProduct(centerToForce, force);
            this->force = this->force + force;
        }
        void Integrate(float dt)
        {
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