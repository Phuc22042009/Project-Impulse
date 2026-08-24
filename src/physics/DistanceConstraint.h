#pragma once
#include "RigidBody2D.h"
#include <vector>
#include <memory>
struct DistanceConstraint
{
    int bodyAIndex;
    int bodyBIndex;

    float restLength;
    float accumulatedImpulse;
};
void WarmStartDistanceConstraint(DistanceConstraint &constraint, std::vector<RigidBody2D> &bodies);
void SolveDistanceConstraint(DistanceConstraint &constraint, std::vector<RigidBody2D> &bodies, float dt);