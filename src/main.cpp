#include <iostream>
#include <vector>
#include <memory>
#include "raylib.h"
#include "math/Vec2.h"
#include "physics/Shape.h"
#include "physics/RigidBody2D.h"
#include "physics/ContactConstraint.h"
#include "physics/BroadPhase.h"
#include "physics/DistanceConstraint.h"

const float FIXED_TIMESTEP = 1.0f / 60.0f;
const Vec2 gravity = Vec2(0.0f, 9.8f * 100.0f);

std::vector<RigidBody2D> bodies;
std::vector<ContactConstraint> newConstraints;
std::vector<ContactConstraint> previousConstraints;
std::vector<DistanceConstraint> distanceConstraints;
std::vector<AABBEntry> entries;
float accumulator = 0.0f;

void Update(float dt)
{
    accumulator += dt;
    while (accumulator >= FIXED_TIMESTEP)
    {
        const int substeps = 4;
        float subDt = FIXED_TIMESTEP / substeps;
        for (int s = 0; s < substeps; s++)
        {
            newConstraints.clear();
            for (auto& body : bodies)
            {
                if (!body.isAwake) continue;
                if (body.invMass == 0) continue;
                body.ApplyForce(gravity * body.mass, body.position);
            }

            for (auto& body : bodies)
                body.IntegrateVelocities(subDt);
            
            auto pairs = BroadPhase(bodies, entries);
            for (const auto& [i, j] : pairs)
            {
                RigidBody2D* a = &bodies[i];
                RigidBody2D* b = &bodies[j];

                Manifold manifold = GetManifold(a, b);
                if (manifold.colliding)
                    for (int p = 0; p < manifold.contactCount; p++)
                        newConstraints.push_back(GenerateContactConstraint(manifold, p, *a, *b, subDt));
            }
            SolveConstraints(newConstraints, previousConstraints, subDt);
            for (DistanceConstraint& dc : distanceConstraints)
                SolveDistanceConstraint(dc, subDt);
            for (auto& body : bodies)
                body.IntegratePositions(subDt);
            previousConstraints = newConstraints;
        }
        
        for (auto& body : bodies)
            UpdateSleep(body, FIXED_TIMESTEP);
        accumulator -= FIXED_TIMESTEP;
    }
}
void Draw()
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    
    
    for (const auto& body : bodies) 
    {
        if (!body.shape) continue;
        if (body.shape->type == ShapeType::Box) 
        {
            const auto& box = static_cast<const Box&>(*body.shape);
            Color color = body.isAwake ? BLUE : GRAY;
            Rectangle rect = {
                body.position.x,
                body.position.y,
                box.width,
                box.height
            };
            DrawRectanglePro(rect, {box.width * 0.5f, box.height * 0.5f}, body.angle * RAD2DEG, color);
        }
        else if (body.shape->type == ShapeType::Circle) 
        {
            const auto& circ = static_cast<const Circle&>(*body.shape);
            Color color = body.isAwake ? RED : GRAY;
            DrawCircle(body.position.x, body.position.y, circ.radius, color);
        }
    }
    EndDrawing();
}
int main()
{
    bodies.reserve(256);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(800, 600, "Physics Engine");
    SetTargetFPS(60);

//RigidBody2D* firstBall = nullptr;

// Newtons cradle setup
for (int i = 0; i < 5; i++) 
{
    auto bobShape = std::make_unique<Circle>();
    bobShape->radius = 10;
    bodies.emplace_back(1.0f, std::move(bobShape));
    RigidBody2D* firstBall = &bodies.back();
    bodies.back().position = Vec2(300 + i * 20, 400);
    bodies.back().restitution = 0.0f;
    bodies.back().staticFriction = 0.0f;
    
    if (i == 0)
        firstBall->position = Vec2(600, 250);
    for (int j = 0; j < 2; j++)
    {
        auto circleShape = std::make_unique<Circle>();
        circleShape->radius = 5;
        bodies.emplace_back(0.0f, std::move(circleShape));
        bodies.back().position = Vec2(300 + i * 20 + (j == 0 ? -10 : 10), 150);

        DistanceConstraint dc;
        dc.bodyA = &bodies.back();
        dc.bodyB = firstBall;
        dc.restLength = (firstBall->position - bodies.back().position).Len();
        dc.accumulatedImpulse = 0;
        distanceConstraints.push_back(dc);
    }
}


    /*Pendulum setup
    auto circleShape = std::make_unique<Circle>();
    circleShape->radius = 10;
    bodies.emplace_back(0.0f, std::move(circleShape));
    bodies.back().position = Vec2(300, 300);
    
    RigidBody2D* prevBob = &bodies.back();
    for (int i = 0; i < 5; i++)
    {
        auto bobShape = std::make_unique<Circle>();
        bobShape->radius = 10;
        bodies.emplace_back(1.0f, std::move(bobShape));
        RigidBody2D* bob = &bodies.back();
        bodies.back().position = Vec2(400, 300 + (i + 1) * 50);
        bodies.back().restitution = 0.0f;
        bodies.back().staticFriction = 0.0f;

        DistanceConstraint dc;
        dc.bodyA = prevBob;
        dc.bodyB = bob;
        dc.restLength = (bob->position - prevBob->position).Len();
        dc.accumulatedImpulse = 0;
        distanceConstraints.push_back(dc);
        prevBob = bob;
    }
    bodies.back().position = Vec2(350, 400);
    */
    /* Box stacking setup
    auto groundShape = std::make_unique<Box>();
    groundShape->width = 800;
    groundShape->height = 50;
    bodies.emplace_back(0.0f, std::move(groundShape));
    bodies.back().position = Vec2(400, 575);
    bodies.back().restitution = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        auto boxShape = std::make_unique<Box>();
        boxShape->width = 50;
        boxShape->height = 50;
        bodies.emplace_back(1, std::move(boxShape));
        bodies.back().position = Vec2(400, 400 - i * 50);
        bodies.back().restitution = 0.5f;
        bodies.back().staticFriction = 0.5f;
    }
    */
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        Update(dt);
        Draw();
    }
    return 0;
}
