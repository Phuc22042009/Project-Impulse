#include <iostream>
#include "physics/Shape.h"
#include "physics/RigidBody2D.h"
#include "raylib.h"
#include "math/Vec2.h"

void DrawRigidBody(const RigidBody2D& body)
{
    Box* box = static_cast<Box*>(body.shape);
    if (!box) return;

    Vector2 size = { box->width, box->height };
    Vector2 origin = { box->width / 2.0f, box->height / 2.0f };
    Rectangle rect = { body.position.x - origin.x, body.position.y - origin.y, box->width, box->height };
    DrawRectanglePro(rect, origin, body.angle * RAD2DEG, BLUE);
}

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "Physics Engine");
    SetTargetFPS(60);

    Box boxShape;
    boxShape.width = 120.0f;
    boxShape.height = 40.0f;

    RigidBody2D boxBodyA(10.0f, &boxShape);
    boxBodyA.position = Vec2(400.0f, 100.0f);
    boxBodyA.angle = 0.0f;

    RigidBody2D boxBodyB(5.0f, &boxShape);
    boxBodyB.position = Vec2(200.0f, 50.0f);
    boxBodyB.angle = 0.0f;

    Vec2 gravity = Vec2(0.0f, 9.81f * 100.0f);
    const float floorY = 560.0f;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt <= 0.0f) dt = 1.0f / 60.0f;

        boxBodyA.ApplyForce(gravity * boxBodyA.mass);
        boxBodyA.Integrate(dt);
        if (boxBodyA.position.y + boxShape.height / 2.0f > floorY)
        {
            boxBodyA.position.y = floorY - boxShape.height / 2.0f;
            boxBodyA.velocity.y = 0.0f;
        }

        boxBodyB.ApplyForce(gravity * boxBodyB.mass);
        boxBodyB.Integrate(dt);
        if (boxBodyB.position.y + boxShape.height / 2.0f > floorY)
        {
            boxBodyB.position.y = floorY - boxShape.height / 2.0f;
            boxBodyB.velocity.y = 0.0f;
        }

        BeginDrawing();
            ClearBackground({20, 22, 28, 255});
            DrawLine(0, (int)floorY, screenWidth, (int)floorY, DARKGRAY);
            DrawRigidBody(boxBodyA);
            DrawRigidBody(boxBodyB);
            DrawText(TextFormat("FPS: %i", GetFPS()), 10, 10, 20, RAYWHITE);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
