#pragma once
#include "../physics/RigidBody2D.h"
#include "Collision.h"
#include "../math/Vec2.h"
#include <vector>
#include <memory>
struct ContactConstraint
{
    int bodyAIndex = -1;
    int bodyBIndex = -1;
    Vec2 rA;
    Vec2 rB;
    Vec2 normal;
    Vec2 contactPoint;
    Vec2 tangent;
    float penetration = 0.0f;
    float accumulatedNormalImpulse = 0.0f;
    float accumulatedFrictionImpulse = 0.0f;

    float accumulatedPseudoImpulse = 0.0f;
    float pseudoBias = 0.0f;

    float bias = 0.0f;
};

void WarmStart(ContactConstraint &constraint, std::vector<RigidBody2D> &bodies);
void MatchAndWarmStart(std::vector<ContactConstraint> &constraints, const std::vector<ContactConstraint> &previousConstraints, std::vector<RigidBody2D> &bodies);
ContactConstraint GenerateContactConstraint(const Manifold &manifold, std::vector<RigidBody2D> &bodies, int pointIndex, int indexA, int indexB, float dt);
void ResolveContactConstraint(ContactConstraint &constraint, std::vector<RigidBody2D> &bodies, float dt);
void SolveRollingResistance(ContactConstraint &constraint, std::vector<RigidBody2D> &bodies);
void SolveContactConstraints(std::vector<ContactConstraint> &constraints, const std::vector<ContactConstraint> &previousConstraints, std::vector<RigidBody2D> &bodies, float dt);
