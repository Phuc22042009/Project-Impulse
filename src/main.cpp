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

                Manifold manifold = GetManifold(&bodies[i], &bodies[j]);
                if (manifold.colliding)
                    for (int p = 0; p < manifold.contactCount; p++)
                        newConstraints.push_back(GenerateContactConstraint(manifold, bodies, p, i, j, subDt));
            }
            SolveConstraints(newConstraints, previousConstraints, bodies, subDt);
            for (int iter = 0; iter < 1; iter++)
            {
                for (DistanceConstraint& dc : distanceConstraints)
                {
                    SolveDistanceConstraint(dc, bodies, subDt);
                }
            }
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
    int bobIndex = (int)bodies.size() - 1;
    bodies[bobIndex].position       = Vec2(300 + i * 20, 400);
    bodies[bobIndex].restitution    = 0.0f;
    bodies[bobIndex].staticFriction = 0.0f;

    for (int j = 0; j < 2; j++)
    {
        auto anchorShape = std::make_unique<Circle>();
        anchorShape->radius = 5;
        bodies.emplace_back(0.0f, std::move(anchorShape));
        int anchorIndex = (int)bodies.size() - 1;   // ← index

        bodies[anchorIndex].position = Vec2(300 + i * 20 + (j == 0 ? -10 : 10), 150);

        DistanceConstraint dc;
        dc.bodyAIndex = anchorIndex;
        dc.bodyBIndex = bobIndex;
        dc.restLength   = (bodies[bobIndex].position - bodies[anchorIndex].position).Len();
        dc.accumulatedImpulse = 0.0f;
        distanceConstraints.push_back(dc);
    }

    if (i == 0)
        bodies[bobIndex].position = Vec2(600, 250);
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
