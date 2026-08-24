#pragma once
#include <vector>
#include <utility>
#include "RigidBody2D.h"
struct AABBEntry
{
    float minX = 0.0f, maxX = 0.0f;
    float minY = 0.0f, maxY = 0.0f;
    int index = -1;
};

void InsertionSort(std::vector<AABBEntry> &entries);
AABBEntry GetAABBEntry(const RigidBody2D &body, int index);
void BroadPhase(const std::vector<RigidBody2D> &bodies, std::vector<AABBEntry> &entries, std::vector<std::pair<int, int>> &outPairs);