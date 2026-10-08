#include "BroadPhase.h"
#include "Shape.h"
#include <algorithm>
#include <cmath>

void InsertionSort(std::vector<AABBEntry> &entries)
{
    for (int i = 1; i < (int)entries.size(); i++)
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
AABBEntry GetAABBEntry(const RigidBody2D &body, int index)
{
    AABBEntry entry;
    entry.index = index;

    if (body.shape->type == ShapeType::Box)
    {
        Box *box = static_cast<Box *>(body.shape.get());
        float cosA = std::abs(std::cos(body.angle));
        float sinA = std::abs(std::sin(body.angle));
        
        float hw = box->width * 0.5f * cosA + box->height * 0.5f * sinA;
        float hh = box->height * 0.5f * cosA + box->width * 0.5f * sinA;

        entry.minX = body.position.x - hw;
        entry.maxX = body.position.x + hw;
        entry.minY = body.position.y - hh;
        entry.maxY = body.position.y + hh;
    }
    else if (body.shape->type == ShapeType::Circle)
    {
        Circle *circle = static_cast<Circle *>(body.shape.get());
        entry.minX = body.position.x - circle->radius;
        entry.maxX = body.position.x + circle->radius;
        entry.minY = body.position.y - circle->radius;
        entry.maxY = body.position.y + circle->radius;
    }

    return entry;
}
void BroadPhase(const std::vector<RigidBody2D> &bodies, std::vector<AABBEntry> &entries, std::vector<std::pair<int, int>> &outPairs)
{
    outPairs.clear();

    if (entries.size() != bodies.size())
    {
        entries.resize(bodies.size());
        for (int i = 0; i < (int)bodies.size(); i++)
        {
            entries[i] = GetAABBEntry(bodies[i], i);
        }
        std::sort(entries.begin(), entries.end(), [](const AABBEntry &a, const AABBEntry &b)
                  { return a.minX < b.minX; });
    }
    else
    {
        for (int i = 0; i < (int)entries.size(); i++)
        {
            entries[i] = GetAABBEntry(bodies[entries[i].index], entries[i].index);
        }
        InsertionSort(entries);
    }


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
                
            outPairs.push_back({indexA, indexB});
        }
    }
}