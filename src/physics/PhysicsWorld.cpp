#include "PhysicsWorld.h"
#include <vector>
#include <algorithm>

PhysicsWorld::PhysicsWorld(const PhysicsSettings &settings) : settings(settings) {}

int PhysicsWorld::AddBody(RigidBody2D &&body)
{
    bodies.emplace_back(std::move(body));
    return static_cast<int>(bodies.size() - 1);
}

void PhysicsWorld::AddDistanceConstraint(const DistanceConstraint &constraint)
{
    distanceConstraints.push_back(constraint);
}

void PhysicsWorld::Step(float dt)
{
    dt = std::min(dt, 0.2f);
    accumulator += dt;

    while (accumulator >= settings.fixedTimestep)
    {
        const float subDt = settings.fixedTimestep / settings.substeps;

        for (int s = 0; s < settings.substeps; s++)
        {
            StepSubstep(subDt);
        }

        for (auto &body : bodies)
        {
            body.UpdateSleep(settings.fixedTimestep);
        }

        PropagateIslandSleep();
        accumulator -= settings.fixedTimestep;
    }
}

const std::vector<RigidBody2D> &PhysicsWorld::GetBodies() const { return bodies; }

Vec2 PhysicsWorld::GetGravity() { return settings.gravity; }
void PhysicsWorld::SetGravity(Vec2 gravity) { settings.gravity = gravity; }

void PhysicsWorld::StepSubstep(float dt)
{
    newContactConstraints.clear();

    ApplyForces();
    IntegrateVelocities(dt);
    GenerateContacts(dt);
    SolveAllConstraints(dt);
    IntegratePositions(dt);

    previousContactConstraints = std::move(newContactConstraints);
}

void PhysicsWorld::ApplyForces()
{
    for (auto &body : bodies)
    {
        if (!body.isAwake)
            continue;
        if (body.invMass == 0)
            continue;
        body.ApplyForce(settings.gravity * body.mass, body.position);
    }
}

void PhysicsWorld::IntegrateVelocities(float dt)
{
    for (auto &body : bodies)
        body.IntegrateVelocities(dt);
}
void PhysicsWorld::GenerateContacts(float dt)
{
    BroadPhase(bodies, entries, broadPhasePairs);

    for (const auto &[i, j] : broadPhasePairs)
    {
        RigidBody2D &ba = bodies[i];
        RigidBody2D &bb = bodies[j];

        if (!ba.isAwake && !bb.isAwake)
            continue;

        Manifold manifold = GetManifold(&bodies[i], &bodies[j]);
        if (!manifold.colliding)
            continue;

        if (ba.isAwake && ba.invMass != 0.0f && !bb.isAwake && bb.invMass != 0.0f)
        {
            bb.WakeUp();
        }
        if (!ba.isAwake && ba.invMass != 0.0f && bb.isAwake && bb.invMass != 0.0f)
        {
            ba.WakeUp();
        }

        if (!ba.isAwake && !bb.isAwake)
            continue;

        for (int p = 0; p < manifold.contactCount; p++)
            newContactConstraints.push_back(GenerateContactConstraint(manifold, bodies, p, i, j, dt));
    }
}
void PhysicsWorld::SolveAllConstraints(float dt)
{
    for (DistanceConstraint &dc : distanceConstraints)
    {
        WarmStartDistanceConstraint(dc, bodies);
    }

    SolveContactConstraints(newContactConstraints, previousContactConstraints, bodies, dt);

    for (int iter = 0; iter < 10; ++iter)
    {
        for (DistanceConstraint &dc : distanceConstraints)
        {
            SolveDistanceConstraint(dc, bodies, dt);
        }
    }
}
void PhysicsWorld::IntegratePositions(float dt)
{
    for (auto &body : bodies)
        body.IntegratePositions(dt);
}
void PhysicsWorld::PropagateIslandSleep()
{

    for (const auto &c : previousContactConstraints)
    {
        RigidBody2D &a = bodies[c.bodyAIndex];
        RigidBody2D &b = bodies[c.bodyBIndex];

        if (a.invMass != 0.0f && b.invMass != 0.0f)
        {
            if (a.isAwake && b.isAwake)
            {
                float minTimer = std::min(a.sleepTimer, b.sleepTimer);
                a.sleepTimer = minTimer;
                b.sleepTimer = minTimer;
            }
        }
    }
}