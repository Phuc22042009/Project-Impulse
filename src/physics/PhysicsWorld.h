#pragma once
#include <vector>
#include "RigidBody2D.h"
#include "Shape.h"
#include "Collision.h"
#include "ContactConstraint.h"
#include "DistanceConstraint.h"
#include "BroadPhase.h"
#include "PhysicsSettings.h"

class PhysicsWorld
{
public:
    explicit PhysicsWorld(const PhysicsSettings &settings = PhysicsSettings());

    int AddBody(RigidBody2D &&body);
    void AddDistanceConstraint(const DistanceConstraint &constraint);
    void Step(float dt);
    
    const std::vector<RigidBody2D> &GetBodies() const;
    const std::vector<AABBEntry> &GetAABBEntries() const { return entries; }
    const std::vector<std::pair<int, int>> &GetBroadPhasePairs() const { return broadPhasePairs; }

    Vec2 GetGravity();
    void SetGravity(Vec2 gravity);

private:
    void StepSubstep(float dt);
    void ApplyForces();
    void IntegrateVelocities(float dt);
    void GenerateContacts(float dt);
    void SolveAllConstraints(float dt);
    void IntegratePositions(float dt);
    void PropagateIslandSleep();

    PhysicsSettings settings;
    float accumulator = 0.0f;

    std::vector<RigidBody2D> bodies;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<ContactConstraint> newContactConstraints;
    std::vector<ContactConstraint> previousContactConstraints;
    std::vector<AABBEntry> entries;
    std::vector<std::pair<int, int>> broadPhasePairs;
};