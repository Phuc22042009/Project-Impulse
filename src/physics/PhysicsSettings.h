#pragma once
#include "../math/Vec2.h"

struct PhysicsSettings
{
    Vec2 gravity = Vec2(0.0f, 9.8f * 100.0f);
    float fixedTimestep = 1.0f / 60.0f;
    int substeps = 2;
};