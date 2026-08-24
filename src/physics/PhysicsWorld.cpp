#include "PhysicsWorld.h"
#include <vector>

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
    accumulator += dt;

    while (accumulator >= settings.fixedTimestep)
    {
        const float subDt = settings.fixedTimestep / settings.substeps;

        for (int s = 0; s < settings.substeps; s++)
        {
            StepSubstep(subDt);
        }

        for (auto &body : bodies)
            body.UpdateSleep(body, settings.fixedTimestep);
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

    previousContactConstraints = newContactConstraints;
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
        Manifold manifold = GetManifold(&bodies[i], &bodies[j]);
        if (manifold.colliding)
            for (int p = 0; p < manifold.contactCount; p++)
                newContactConstraints.push_back(GenerateContactConstraint(manifold, bodies, p, i, j, dt));
    }
}
void PhysicsWorld::SolveAllConstraints(float dt)
{
    SolveContactConstraints(newContactConstraints, previousContactConstraints, bodies, dt);

    for (DistanceConstraint &dc : distanceConstraints)
    {
        SolveDistanceConstraint(dc, bodies, dt);
    }
}
void PhysicsWorld::IntegratePositions(float dt)
{
    for (auto &body : bodies)
        body.IntegratePositions(dt);
}