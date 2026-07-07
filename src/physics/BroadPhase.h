#pragma once
#include <vector>
#include "RigidBody2D.h"
#include <memory>
struct AABBEntry
{
    float minX = 0, maxX = 0;
    float minY = 0, maxY = 0;
    int index;
};
inline void InsertionSort(std::vector<AABBEntry> &entries)
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
inline AABBEntry GetAABBEntry(const RigidBody2D *body, int index)
{
    AABBEntry entry;
    entry.index = index;

    if (!body || !body->shape)
        return entry;

    if (body->shape->type == ShapeType::Box)
    {
        Box *box = static_cast<Box *>(body->shape.get());
        float cosA = std::abs(std::cos(body->angle));
        float sinA = std::abs(std::sin(body->angle));
        // Calculate half-width and half-height of the AABB
        float hw = box->width * 0.5f * cosA + box->height * 0.5f * sinA;
        float hh = box->height * 0.5f * cosA + box->width * 0.5f * sinA;

        entry.minX = body->position.x - hw;
        entry.maxX = body->position.x + hw;
        entry.minY = body->position.y - hh;
        entry.maxY = body->position.y + hh;
    }
    else if (body->shape->type == ShapeType::Circle)
    {
        Circle *circle = static_cast<Circle *>(body->shape.get());
        entry.minX = body->position.x - circle->radius;
        entry.maxX = body->position.x + circle->radius;
        entry.minY = body->position.y - circle->radius;
        entry.maxY = body->position.y + circle->radius;
    }

    return entry;
}
inline std::vector<std::pair<int, int>> BroadPhase(const std::vector<RigidBody2D> &bodies, std::vector<AABBEntry> &entries)
{
    entries.resize(bodies.size());
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        entries[i] = GetAABBEntry(&bodies[i], i);
    }
    if (bodies.size() < 100)
        InsertionSort(entries);
    else
        std::sort(entries.begin(), entries.end(), [](const AABBEntry &a, const AABBEntry &b)
                  { return a.minX < b.minX; });

    std::vector<std::pair<int, int>> pairs;
    for (int i = 0; i < (int)entries.size(); i++)
    {
        for (int j = i + 1; j < (int)entries.size(); j++)
        {
            if (entries[j].minX > entries[i].maxX)
                break;

            if (entries[i].maxY < entries[j].minY || entries[i].minY > entries[j].maxY)
                continue;

            int indexA = entries[i].index;
            int indexB = entries[j].index;

            if (bodies[indexA].invMass == 0 && bodies[indexB].invMass == 0)
                continue;
            if (!bodies[indexA].isAwake && !bodies[indexB].isAwake)
                continue;
            pairs.push_back({indexA, indexB});
        }
    }
    return pairs;
}