#include <iostream>
#include "physics/Shape.h"
#include "physics/RigidBody2D.h"
#include "raylib.h"
#include "math/Vec2.h"
#include <vector>
#include "physics/ContactConstraint.h"
#include "physics/BroadPhase.h"
#include "physics/DistanceConstraint.h"

const float FIXED_TIMESTEP = 1.0f / 60.0f;
const Vec2 gravity = Vec2(0.0f, 9.8f * 100.0f);

std::vector<RigidBody2D*> bodies;
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
            for (RigidBody2D* body : bodies)
            {
                if (!body->isAwake) continue;
                if (body->invMass == 0) continue;
                body->ApplyForce(gravity * body->mass, body->position);
            }

            for (RigidBody2D* body : bodies)
                body->IntegrateVelocities(subDt);

            std::vector<ContactConstraint> newConstraints;
            auto pairs = BroadPhase(bodies, entries);
            for (const auto& [i, j] : pairs)
            {
                RigidBody2D* a = bodies[i];
                RigidBody2D* b = bodies[j];

                Manifold manifold = GetManifold(*a, *b);
                if (manifold.colliding)
                    for (int p = 0; p < manifold.contactCount; p++)
                        newConstraints.push_back(GenerateContactConstraint(manifold, p, *a, *b, subDt));
            }
            SolveConstraints(newConstraints, previousConstraints, subDt);
            for (DistanceConstraint& dc : distanceConstraints)
                SolveDistanceConstraint(dc, subDt);
            for (RigidBody2D* body : bodies)
                body->IntegratePositions(subDt);
            previousConstraints = newConstraints;
        }
        
        for (RigidBody2D* body : bodies)
            UpdateSleep(*body, FIXED_TIMESTEP);
        accumulator -= FIXED_TIMESTEP;
    }
}
void Draw()
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    
    
    for (const RigidBody2D* body : bodies) 
    {
        Box* box = dynamic_cast<Box*>(body->shape);
        Circle* circ = dynamic_cast<Circle*>(body->shape);

        if (box)
        {
            Rectangle rect = {
                body->position.x,
                body->position.y,
                box->width,
                box->height
            };
            Color color = body->isAwake ? BLUE : GRAY;
            DrawRectanglePro(rect, {box->width * 0.5f, box->height * 0.5f}, body->angle * RAD2DEG, color);
        }
        else if (circ) 
        {
            Color color = body->isAwake ? RED : GRAY;
            DrawCircle(body->position.x, body->position.y, circ->radius, color);
        }
    }
    EndDrawing();
}

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(800, 600, "Physics Engine");
    SetTargetFPS(60);

//RigidBody2D* firstBall = nullptr;

// Newtons cradle setup
/*for (int i = 0; i < 5; i++) 
{
    Circle* bobShape = new Circle();
    bobShape->radius = 10;
    RigidBody2D* bob = new RigidBody2D(1, bobShape);
    bob->position = Vec2(300 + i * 20, 400);
    bob->restitution = 0.0f;
    bob->staticFriction = 0.0f;
    bodies.push_back(bob);
    if (i == 0) firstBall = bob;

    for (int j = 0; j < 2; j++)
    {
        Circle* circleShape = new Circle();
        circleShape->radius = 5;
        RigidBody2D* anchor = new RigidBody2D(0, circleShape);
        anchor->position = Vec2(300 + i * 20 + (j == 0 ? -10 : 10), 150);
        bodies.push_back(anchor);

        DistanceConstraint dc;
        dc.bodyA = anchor;
        dc.bodyB = bob;
        dc.restLength = (bob->position - anchor->position).Len();
        dc.accumulatedImpulse = 0;
        distanceConstraints.push_back(dc);
    }
}
if (firstBall)
    firstBall->position = Vec2(200, 250);
*/

    /*Pendulum setup
    Circle* circleShape = new Circle();
        circleShape->radius = 10;
        RigidBody2D* anchor = new RigidBody2D(0, circleShape);
        anchor->position = Vec2(300, 300);
        bodies.push_back(anchor);
    
    RigidBody2D* prevBob = anchor;
    for (int i = 0; i < 5; i++)
    {
        Circle* bobShape = new Circle();
        bobShape->radius = 10;
        RigidBody2D* bob = new RigidBody2D(1, bobShape);
        bob->position = Vec2(400, 300 + (i + 1) * 50);
        bob->restitution = 0.0f;
        bob->staticFriction = 0.0f;
        bodies.push_back(bob);

        DistanceConstraint dc;
        dc.bodyA = prevBob;
        dc.bodyB = bob;
        dc.restLength = (bob->position - prevBob->position).Len();
        dc.accumulatedImpulse = 0;
        distanceConstraints.push_back(dc);
        prevBob = bob;
    }
    bodies.back()->position = Vec2(350, 400);
    */
    // Box stacking setup
    Box* groundShape = new Box();
    groundShape->width = 800;
    groundShape->height = 50;
    RigidBody2D* ground = new RigidBody2D(0, groundShape);
    ground->position = Vec2(400, 575);
    ground->restitution = 0.0f;
    ground->staticFriction = 0.5f;
    bodies.push_back(ground);

    for (int i = 0; i < 3; i++)
    {
        Box* boxShape = new Box();
        boxShape->width = 50;
        boxShape->height = 50;
        RigidBody2D* box = new RigidBody2D(1, boxShape);
        box->position = Vec2(400, 400 - i * 50);
        box->restitution = 0.5f;
        box->staticFriction = 0.5f;
        bodies.push_back(box);
    }
    
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        Update(dt);
        Draw();
    }

    for (RigidBody2D* body : bodies) 
    {
        delete body->shape;
        delete body;
    }

    CloseWindow();
    return 0;
}
