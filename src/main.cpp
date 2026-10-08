#include "physics/PhysicsWorld.h"
#include "raylib.h"
#include <memory>
#include <algorithm>
#include <iostream>

enum class SpawnShape
{
    Box,
    Circle
};

class PhysicsApp
{
public:
    PhysicsApp() : world(PhysicsSettings()) {}

    void Run()
    {
        SetConfigFlags(FLAG_MSAA_4X_HINT);
        InitWindow(1024, 768, "2D Rigid Body Physics Engine - Demo");
        SetTargetFPS(60);


        CreateFloor();

        while (!WindowShouldClose())
        {
            float dt = GetFrameTime();

            ProcessInput();
            world.Step(dt);



            BeginDrawing();
            ClearBackground(RAYWHITE);

            RenderPhysics();
            DrawFPS(20, 20);

            int totalBodies = (int)world.GetBodies().size();
            DrawText(TextFormat("Body Count: %d / %d", totalBodies, MAX_STRESS_BODIES), 20, 50, 20, DARKGRAY);

            if (isStressTesting)
            {
                DrawText("STRESS TEST: RUNNING (Press [S] to Stop)", 20, 80, 16, MAROON);
            }
            else
            {
                DrawText("Press [S] to Start 500 Boxes Stress Test", 20, 80, 16, DARKGRAY);
            }

            EndDrawing();
        }

        CloseWindow();
    }

private:
    PhysicsWorld world;
    SpawnShape currentSpawnShape = SpawnShape::Box;
    Vec2 dragStart{0.0f, 0.0f};
    bool isDragging = false;
    float spawnSize = 30.0f;
    float restitution = 0.4f;
    float friction = 0.5f;

    bool isStressTesting = false;
    const int MAX_STRESS_BODIES = 200;

    void CreateFloor()
    {
        auto boxShape = std::make_unique<Box>(1024.0f, 50.0f);
        RigidBody2D floor(0.0f, std::move(boxShape));
        floor.position = Vec2(512.0f, 740.0f);
        floor.restitution = 0.2f;
        floor.staticFriction = 0.8f;
        world.AddBody(std::move(floor));
    }

    void SpawnBody(Vec2 position, Vec2 velocity)
    {
        if (currentSpawnShape == SpawnShape::Box)
        {
            auto boxShape = std::make_unique<Box>(spawnSize, spawnSize);
            RigidBody2D body(1.0f, std::move(boxShape));
            body.position = position;
            body.velocity = velocity;
            body.restitution = restitution;
            body.staticFriction = friction;
            world.AddBody(std::move(body));
        }
        else
        {
            auto circleShape = std::make_unique<Circle>(spawnSize * 0.5f);
            RigidBody2D body(1.0f, std::move(circleShape));
            body.position = position;
            body.velocity = velocity;
            body.restitution = restitution;
            body.staticFriction = friction;
            world.AddBody(std::move(body));
        }
    }

    void ProcessInput()
    {
        if (IsKeyPressed(KEY_S))
        {
            isStressTesting = !isStressTesting;
        }

        if (isStressTesting && world.GetBodies().size() < (size_t)MAX_STRESS_BODIES)
        {
            for (int i = 0; i < 3 && world.GetBodies().size() < (size_t)MAX_STRESS_BODIES; ++i)
            {
                float randomX = (float)GetRandomValue(100, 924);
                float randomY = (float)GetRandomValue(50, 150);
                Vec2 randomVel((float)GetRandomValue(-20, 20), (float)GetRandomValue(0, 50));

                SpawnBody(Vec2(randomX, randomY), randomVel);
            }
        }

        float scroll = GetMouseWheelMove();
        if (scroll != 0.0f)
        {
            spawnSize = std::clamp(spawnSize + scroll * 5.0f, 10.0f, 150.0f);
        }

        Vec2 mousePos(GetMouseX(), GetMouseY());

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            isDragging = true;
            dragStart = mousePos;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && isDragging)
        {
            Vec2 dragDelta = mousePos - dragStart;
            Vec2 velocity = dragDelta * 4.0f;
            SpawnBody(dragStart, velocity);
            isDragging = false;
        }
    }

    void RenderPhysics()
    {
        for (const auto &body : world.GetBodies())
        {
            if (!body.shape)
                continue;

            Color color = body.isAwake ? DARKBLUE : GRAY;

            if (body.shape->type == ShapeType::Box)
            {
                const auto &box = static_cast<const Box &>(*body.shape);
                Rectangle rect = {body.position.x, body.position.y, box.width, box.height};
                DrawRectanglePro(rect, {box.width * 0.5f, box.height * 0.5f}, body.angle * RAD2DEG, color);
            }
            else if (body.shape->type == ShapeType::Circle)
            {
                const auto &circ = static_cast<const Circle &>(*body.shape);
                DrawCircleV({body.position.x, body.position.y}, circ.radius, color);

                Vec2 dir(std::cos(body.angle) * circ.radius, std::sin(body.angle) * circ.radius);
                DrawLineV({body.position.x, body.position.y}, {body.position.x + dir.x, body.position.y + dir.y}, WHITE);
            }
        }

        if (isDragging)
        {
            Vec2 mousePos(GetMouseX(), GetMouseY());
            DrawCircleV({dragStart.x, dragStart.y}, 5.0f, RED);
            DrawLineV({dragStart.x, dragStart.y}, {mousePos.x, mousePos.y}, RED);
        }
    }
    void DebugPrintEveryFrame()
    {
        static int frameCount = 0;
        frameCount++;

        std::cout << "=== FRAME " << frameCount << " ===\n";

        const auto &bodies = world.GetBodies();
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            std::cout << "Box #" << i << " | Pos: (" << bodies[i].position.x << ", " << bodies[i].position.y << ")\n";
        }
    }
};

int main()
{
    PhysicsApp app;
    app.Run();
    return 0;
}