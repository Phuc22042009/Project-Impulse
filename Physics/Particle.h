#pragma once
#include "../Math/Vec2.h"
#define NUM_PARTICLES 1
#include <iostream>
struct Particle
{
    Vec2 position;
    Vec2 velocity;
    float mass;
};

Particle particles[NUM_PARTICLES];

void PrintParticles() 
{
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle* particle = &particles[i];
        printf("particle[%i] (%.2f, %.2f)\n", i, particle->position.x, particle->position.y);
    }
}
void InitializeParticles() 
{
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].position = Vec2(float(rand() % 51), float(rand() % 51));
        particles[i].velocity = Vec2(0,0);
        particles[i].mass = 1;
    }
}
Vec2 ComputeForce(Particle* particle) 
{
    return Vec2(0, particle->mass * -9.81);
}
void RunSimulation() 
{
    float totalTime = 10;
    float currentTime = 0;
    float dt = 1;

    InitializeParticles();
    PrintParticles();

    while (currentTime < totalTime)
    {
        _sleep(dt);

        for (int i = 0; i < NUM_PARTICLES; i++)
        {
            Particle* particle = &particles[i];
            Vec2 force = ComputeForce(particle);
            Vec2 acceleration = Vec2(force.x/particle->mass, force.y/particle->mass);
            particle->velocity.x += acceleration.x * dt;
            particle->velocity.y += acceleration.y * dt;

            particle->position.x += particle->velocity.x * dt;
            particle->position.y += particle->velocity.y * dt;
        }

        PrintParticles();
        currentTime += dt;
    }
}
