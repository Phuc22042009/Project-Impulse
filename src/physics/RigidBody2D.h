#pragma once
#include "../math/Vec2.h"
#include "Shape.h"
#include <memory>

struct RigidBody2D
{
    std::unique_ptr<Shape> shape;

    Vec2 position;
    Vec2 force;
    Vec2 velocity;

    Vec2 pseudoVelocity;
    float pseudoAngularVelocity = 0.0f;
    float angularVelocity = 0.0f;
    float angle = 0.0f;
    
    float mass = 0.0f;
    float invMass = 0.0f;
    
    float torque = 0.0f;
    
    float inertia = 0.0f;
    float invInertia = 0.0f;
    float restitution = 0.0f;
    float staticFriction = 0.5f;

    bool isAwake = true;
    float sleepTimer = 0.0f;

    RigidBody2D(float mass, std::unique_ptr<Shape> shape);

    void ApplyForce(const Vec2 &force, const Vec2 &impactPoint);
    void IntegrateVelocities(float dt);
    void IntegratePositions(float dt);
    void UpdateSleep(float dt);
    void WakeUp();
};
