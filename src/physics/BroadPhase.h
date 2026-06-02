#pragma once
#include <vector>
#include "RigidBody2D.h"

struct AABBEntry
{
    float minX, maxX;
    float minY, maxY;
    int index;
};
void InsertionSort(std::vector<AABBEntry>& entries) 
{
    for (int i = 1; i < entries.size(); i++) 
    {
        AABBEntry key = entries[i];
        int j = i - 1;

        while (j >= 0 && entries[j].minX > key.minX) 
        {
            entries[j + 1] = entries[j];
            j--;
        }
        entries[j + 1] = key;
    }
}
AABBEntry GetAABBEntry(RigidBody2D* body, int index) 
{
    AABBEntry entry;
    entry.index = index;

    if (!body->shape) return entry;
    
    Box* box = dynamic_cast<Box*>(body->shape);
    Circle* circle = dynamic_cast<Circle*>(body->shape);

    if (box) 
    {
        float cosA = std::abs(std::cos(body->angle));
        float sinA = std::abs(std::sin(body->angle));
        float hw = box->width * 0.5f * cosA + box->height * 0.5f * sinA;
        float hh = box->height * 0.5f * cosA + box->width * 0.5f * sinA;
        entry.minX = body->position.x - hw;
        entry.maxX = body->position.x + hw;
        entry.minY = body->position.y - hh;
        entry.maxY = body->position.y + hh;
    } 
    else if (circle) 
    {
        entry.minX = body->position.x - circle->radius;
        entry.maxX = body->position.x + circle->radius;
        entry.minY = body->position.y - circle->radius;
        entry.maxY = body->position.y + circle->radius;
    }
    
    return entry;
}
std::vector<std::pair<int, int>> BroadPhase(const std::vector<RigidBody2D*>& bodies, std::vector<AABBEntry>& entries) 
{
    entries.resize(bodies.size());
    for (int i = 0; i < (int)bodies.size(); i++) 
    {
        entries[i] = GetAABBEntry(bodies[i], i);
    }

    InsertionSort(entries);
    
    std::vector<std::pair<int, int>> pairs;
    for (int i = 0; i < (int)entries.size(); i++) 
    {
        for (int j = i + 1; j < (int)entries.size(); j++) 
        {
            if (entries[j].minX > entries[i].maxX) break;

            if (entries[i].maxY < entries[j].minY || entries[i].minY > entries[j].maxY) continue;

            int indexA = entries[i].index;
            int indexB = entries[j].index;
            
            if (bodies[indexA]->invMass == 0 && bodies[indexB]->invMass == 0) continue;
            if (!bodies[indexA]->isAwake && !bodies[indexB]->isAwake) continue;
            pairs.push_back({indexA, indexB});
        }
    }
    return pairs;
}