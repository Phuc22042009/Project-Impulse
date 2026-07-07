#pragma once

#include <vector>
#include "RigidBody2D.h"
#include "Shape.h"
#include "Collision.h"
#include "ContactConstraint.h"
#include "DistanceConstraint.h"
#include "BroadPhase.h"
#include "PhysicsSettings.h"
#include <utility>

class PhysicsWorld
{
public:
    explicit PhysicsWorld(const PhysicsSettings &settings = PhysicsSettings()) : settings(settings) {}

    int AddBody(RigidBody2D &&body)
    {
        bodies.emplace_back(std::move(body));
        return static_cast<int>(bodies.size() - 1);
    }

    void AddDistanceConstraint(const DistanceConstraint &constraint)
    {
        distanceConstraints.push_back(constraint);
    }

    void Step(float dt)
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
                UpdateSleep(body, settings.fixedTimestep);
            accumulator -= settings.fixedTimestep;
        }
    }

    std::vector<RigidBody2D> &GetBodies() { return bodies; }
    const std::vector<RigidBody2D> &GetBodies() const { return bodies; }

    Vec2 GetGravity() {return settings.gravity;}
    void SetGravity(Vec2 gravity) {settings.gravity = gravity;}
private:
    void StepSubstep(float dt)
    {
        newContactConstraints.clear();

        ApplyForces();
        IntegrateVelocities(dt);
        GenerateContacts(dt);
        SolveAllConstraints(dt);
        IntegratePositions(dt);

        previousContactConstraints = newContactConstraints;
    }

    void ApplyForces()
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

    void IntegrateVelocities(float dt)
    {
        for (auto &body : bodies)
            body.IntegrateVelocities(dt);
    }
    void GenerateContacts(float dt)
    {
        auto pairs = BroadPhase(bodies, entries);

        for (const auto &[i, j] : pairs)
        {
            Manifold manifold = GetManifold(&bodies[i], &bodies[j]);
            if (manifold.colliding)
                for (int p = 0; p < manifold.contactCount; p++)
                    newContactConstraints.push_back(GenerateContactConstraint(manifold, bodies, p, i, j, dt));
        }
    }
    void SolveAllConstraints(float dt)
    {
        SolveContactConstraints(newContactConstraints, previousContactConstraints, bodies, dt);

        for (DistanceConstraint &dc : distanceConstraints)
        {
            SolveDistanceConstraint(dc, bodies, dt);
        }
    }
    void IntegratePositions(float dt)
    {
        for (auto &body : bodies)
            body.IntegratePositions(dt);
    }

private:
    PhysicsSettings settings;

    std::vector<RigidBody2D> bodies;
    std::vector<ContactConstraint> newContactConstraints;
    std::vector<ContactConstraint> previousContactConstraints;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<AABBEntry> entries;

    float accumulator = 0.0f;
};