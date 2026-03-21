#pragma once
#include <iostream>
#include "../Math/Vec2.h"
#define NUM_RIGID_BODIES 1 

struct BoxShape 
{
    float width;
    float height;
    float mass;
    float momentOfInertia;
};

void CalculateBoxInertia(BoxShape* boxShape) {
    float m = boxShape->mass;
    float h = boxShape->height;
    float w = boxShape->width;
    boxShape->momentOfInertia = m * (h * h + w*w) / 12;
}

struct RigidBody
{
    Vec2 position;
    Vec2 linearVelocity;
    float angle;
    float angularVelocity;
    Vec2 Force;
    float torque;
    BoxShape shape;
};

RigidBody _rigidBodies[NUM_RIGID_BODIES];

void PrintRigidBodies() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody* rigidBody = &_rigidBodies[i];
        printf("body[%i] position = (%.2f, %.2f), angle = %.2f\n", i, rigidBody->position.x, rigidBody->position.y, rigidBody->angle);
    }
}

void InitializeRigidBodies() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody* rigidBody = &_rigidBodies[i];

        rigidBody->position = Vec2(float(rand() % 51), float(rand() % 51));
        rigidBody->angle = float(rand() % 361)/360.f * 3.14 * 2;
        rigidBody->linearVelocity = Vec2(0,0);
        rigidBody->angularVelocity = 0;

        BoxShape shape;
        shape.mass = 10;
        shape.width = 1 + float(rand() % 3);
        shape.height = 1 + float(rand() % 3);
        CalculateBoxInertia(&shape);
        rigidBody->shape = shape;
    }
}
void ComputeForceAndTorque(RigidBody* rigidBody) {
    Vec2 force = Vec2(0,100);
    rigidBody->Force = force;

    Vec2 r = Vec2(rigidBody->shape.width / 2, rigidBody->shape.height / 2);
    rigidBody->torque = r.x * force.y -r.y * force.x;
}
void RunRigidBodySimulation() {
    float totalSimulationTime = 10;
    float currentTime = 0;
    float dt = 1;

    InitializeRigidBodies();
    PrintRigidBodies();

    while (currentTime < totalSimulationTime)
    {
        _sleep(dt);

        for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
            RigidBody* rigidBody = &_rigidBodies[i];
            ComputeForceAndTorque(rigidBody);
            Vec2 linearAcceleration = Vec2(rigidBody->Force.x / rigidBody->shape.mass, rigidBody->Force.y / rigidBody->shape.mass);
            rigidBody->linearVelocity.x += linearAcceleration.x * dt;
            rigidBody->linearVelocity.y += linearAcceleration.y * dt;
            rigidBody->position.x += rigidBody->linearVelocity.x * dt;
            rigidBody->position.y += rigidBody->linearVelocity.y * dt;
            float angularAcceleration = rigidBody->torque / rigidBody->shape.momentOfInertia;
            rigidBody->angularVelocity += angularAcceleration * dt;
            rigidBody->angle += rigidBody->angularVelocity * dt;
        }
        PrintRigidBodies();
        currentTime += dt;
    }
}