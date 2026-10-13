#include "physics/PhysicsWorld.h"
#include "raylib.h"
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

enum class SpawnShape
{
    Box,
    Circle
};

enum class DemoScene
{
    Stacking,
    Constraints,
    RestitutionFriction,
    StressTest
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

        LoadScene(DemoScene::Stacking);

        while (!WindowShouldClose())
        {
            float dt = GetFrameTime();

            ProcessInput();
            world.Step(dt);

            BeginDrawing();
            ClearBackground(RAYWHITE);

            RenderPhysics();
            RenderUI();

            EndDrawing();
        }

        CloseWindow();
    }

private:
    PhysicsWorld world;
    DemoScene currentScene = DemoScene::Stacking;
    SpawnShape currentSpawnShape = SpawnShape::Box;
    std::vector<DistanceConstraint> activeConstraints;

    Vec2 dragStart{0.0f, 0.0f};
    bool isDragging = false;
    float spawnSize = 35.0f;
    float defaultRestitution = 0.4f;
    float defaultFriction = 0.5f;

    bool isStressTesting = false;
    bool showBroadPhaseDebug = false;
    const int MAX_STRESS_BODIES = 325;

    void CreateFloor()
    {
        auto boxShape = std::make_unique<Box>(1024.0f, 50.0f);
        RigidBody2D floor(0.0f, std::move(boxShape));
        floor.position = Vec2(512.0f, 740.0f);
        floor.restitution = 0.2f;
        floor.staticFriction = 0.8f;
        world.AddBody(std::move(floor));
    }

    void AddDistanceJoint(int bodyA, int bodyB, float length)
    {
        DistanceConstraint dc;
        dc.bodyAIndex = bodyA;
        dc.bodyBIndex = bodyB;
        dc.restLength = length;
        dc.accumulatedImpulse = 0.0f;
        world.AddDistanceConstraint(dc);
        activeConstraints.push_back(dc);
    }

    void LoadScene(DemoScene scene)
    {
        currentScene = scene;
        world = PhysicsWorld(PhysicsSettings());
        activeConstraints.clear();
        isStressTesting = (scene == DemoScene::StressTest);

        CreateFloor();

        if (scene == DemoScene::Stacking)
        {
            float boxSize = 36.0f;
            float startX = 340.0f;
            for (int i = 0; i < 8; ++i)
            {
                auto shape = std::make_unique<Box>(boxSize, boxSize);
                RigidBody2D body(1.0f, std::move(shape));
                body.position = Vec2(startX, 700.0f - i * (boxSize + 2.0f));
                body.restitution = 0.0f;
                body.staticFriction = 0.6f;
                world.AddBody(std::move(body));
            }

            float pyrX = 660.0f;
            int levels = 4;
            float spacing = boxSize + 3.0f;
            for (int row = 0; row < levels; ++row)
            {
                int count = levels - row;
                float rowStartX = pyrX - (count - 1) * spacing * 0.5f;
                float y = 700.0f - row * (boxSize + 2.0f);
                for (int col = 0; col < count; ++col)
                {
                    auto shape = std::make_unique<Box>(boxSize, boxSize);
                    RigidBody2D body(1.0f, std::move(shape));
                    body.position = Vec2(rowStartX + col * spacing, y);
                    body.restitution = 0.0f;
                    body.staticFriction = 0.6f;
                    world.AddBody(std::move(body));
                }
            }
        }
        else if (scene == DemoScene::Constraints)
        {
            auto anchorLeft = std::make_unique<Box>(24.0f, 24.0f);
            RigidBody2D bLeft(0.0f, std::move(anchorLeft));
            bLeft.position = Vec2(180.0f, 280.0f);
            int prevIdx = world.AddBody(std::move(bLeft));

            int plankCount = 11;
            float plankW = 46.0f;
            float plankH = 14.0f;
            float stepX = (844.0f - 180.0f) / (plankCount + 1);

            for (int i = 1; i <= plankCount; ++i)
            {
                auto plank = std::make_unique<Box>(plankW, plankH);
                RigidBody2D bPlank(1.0f, std::move(plank));
                bPlank.position = Vec2(180.0f + i * stepX, 280.0f);
                bPlank.restitution = 0.0f;
                bPlank.staticFriction = 0.7f;
                int currIdx = world.AddBody(std::move(bPlank));

                AddDistanceJoint(prevIdx, currIdx, stepX);
                prevIdx = currIdx;
            }

            auto anchorRight = std::make_unique<Box>(24.0f, 24.0f);
            RigidBody2D bRight(0.0f, std::move(anchorRight));
            bRight.position = Vec2(844.0f, 280.0f);
            int rightIdx = world.AddBody(std::move(bRight));
            AddDistanceJoint(prevIdx, rightIdx, stepX);

            auto pendAnchor = std::make_unique<Circle>(10.0f);
            RigidBody2D bPendA(0.0f, std::move(pendAnchor));
            bPendA.position = Vec2(512.0f, 80.0f);
            int p0 = world.AddBody(std::move(bPendA));

            auto pendBall1 = std::make_unique<Circle>(18.0f);
            RigidBody2D bPend1(2.0f, std::move(pendBall1));
            bPend1.position = Vec2(580.0f, 80.0f);
            bPend1.restitution = 0.6f;
            int p1 = world.AddBody(std::move(bPend1));
            AddDistanceJoint(p0, p1, 70.0f);

            auto pendBall2 = std::make_unique<Circle>(18.0f);
            RigidBody2D bPend2(2.0f, std::move(pendBall2));
            bPend2.position = Vec2(650.0f, 80.0f);
            bPend2.restitution = 0.6f;
            int p2 = world.AddBody(std::move(bPend2));
            AddDistanceJoint(p1, p2, 70.0f);
        }
        else if (scene == DemoScene::RestitutionFriction)
        {
            auto rampShape = std::make_unique<Box>(460.0f, 20.0f);
            RigidBody2D ramp(0.0f, std::move(rampShape));
            ramp.position = Vec2(300.0f, 480.0f);
            ramp.angle = 24.0f * DEG2RAD;
            ramp.staticFriction = 0.6f;
            world.AddBody(std::move(ramp));

            float frictions[3] = {0.05f, 0.35f, 0.85f};
            for (int i = 0; i < 3; ++i)
            {
                auto box = std::make_unique<Box>(32.0f, 32.0f);
                RigidBody2D b(1.0f, std::move(box));
                b.position = Vec2(180.0f + i * 90.0f, 350.0f - i * 40.0f);
                b.angle = 24.0f * DEG2RAD;
                b.staticFriction = frictions[i];
                world.AddBody(std::move(b));
            }

            float restitutions[4] = {0.0f, 0.4f, 0.75f, 0.98f};
            for (int i = 0; i < 4; ++i)
            {
                auto circ = std::make_unique<Circle>(20.0f);
                RigidBody2D b(1.0f, std::move(circ));
                b.position = Vec2(620.0f + i * 85.0f, 200.0f);
                b.restitution = restitutions[i];
                b.staticFriction = 0.3f;
                world.AddBody(std::move(b));
            }
        }
    }

    void SpawnBody(Vec2 position, Vec2 velocity)
    {
        if (currentSpawnShape == SpawnShape::Box)
        {
            auto boxShape = std::make_unique<Box>(spawnSize, spawnSize);
            RigidBody2D body(1.0f, std::move(boxShape));
            body.position = position;
            body.velocity = velocity;
            body.restitution = defaultRestitution;
            body.staticFriction = defaultFriction;
            world.AddBody(std::move(body));
        }
        else
        {
            auto circleShape = std::make_unique<Circle>(spawnSize * 0.5f);
            RigidBody2D body(1.0f, std::move(circleShape));
            body.position = position;
            body.velocity = velocity;
            body.restitution = defaultRestitution;
            body.staticFriction = defaultFriction;
            world.AddBody(std::move(body));
        }
    }

    void ProcessInput()
    {
        if (IsKeyPressed(KEY_ONE)) LoadScene(DemoScene::Stacking);
        if (IsKeyPressed(KEY_TWO)) LoadScene(DemoScene::Constraints);
        if (IsKeyPressed(KEY_THREE)) LoadScene(DemoScene::RestitutionFriction);
        if (IsKeyPressed(KEY_FOUR)) LoadScene(DemoScene::StressTest);
        if (IsKeyPressed(KEY_R)) LoadScene(currentScene);
        if (IsKeyPressed(KEY_B)) showBroadPhaseDebug = !showBroadPhaseDebug;

        if (IsKeyPressed(KEY_SPACE))
        {
            currentSpawnShape = (currentSpawnShape == SpawnShape::Box) ? SpawnShape::Circle : SpawnShape::Box;
        }

        if (IsKeyPressed(KEY_S))
        {
            isStressTesting = !isStressTesting;
        }

        if (isStressTesting && world.GetBodies().size() < (size_t)MAX_STRESS_BODIES)
        {
            for (int i = 0; i < 3 && world.GetBodies().size() < (size_t)MAX_STRESS_BODIES; ++i)
            {
                float randomX = (float)GetRandomValue(120, 904);
                float randomY = (float)GetRandomValue(40, 160);
                Vec2 randomVel((float)GetRandomValue(-25, 25), (float)GetRandomValue(10, 60));

                if (GetRandomValue(0, 1) == 0)
                {
                    auto shape = std::make_unique<Box>(28.0f, 28.0f);
                    RigidBody2D b(1.0f, std::move(shape));
                    b.position = Vec2(randomX, randomY);
                    b.velocity = randomVel;
                    b.restitution = 0.3f;
                    b.staticFriction = 0.5f;
                    world.AddBody(std::move(b));
                }
                else
                {
                    auto shape = std::make_unique<Circle>(14.0f);
                    RigidBody2D b(1.0f, std::move(shape));
                    b.position = Vec2(randomX, randomY);
                    b.velocity = randomVel;
                    b.restitution = 0.4f;
                    b.staticFriction = 0.4f;
                    world.AddBody(std::move(b));
                }
            }
        }

        float scroll = GetMouseWheelMove();
        if (scroll != 0.0f)
        {
            spawnSize = std::clamp(spawnSize + scroll * 5.0f, 15.0f, 120.0f);
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
            Vec2 velocity = dragDelta * 4.5f;
            SpawnBody(dragStart, velocity);
            isDragging = false;
        }
    }

    void RenderPhysics()
    {
        const auto &bodies = world.GetBodies();

        for (const auto &dc : activeConstraints)
        {
            if (dc.bodyAIndex < (int)bodies.size() && dc.bodyBIndex < (int)bodies.size())
            {
                Vec2 pA = bodies[dc.bodyAIndex].position;
                Vec2 pB = bodies[dc.bodyBIndex].position;
                DrawLineEx({pA.x, pA.y}, {pB.x, pB.y}, 3.5f, DARKGREEN);
                DrawCircleV({pA.x, pA.y}, 4.0f, DARKBLUE);
                DrawCircleV({pB.x, pB.y}, 4.0f, DARKBLUE);
            }
        }

        for (const auto &body : bodies)
        {
            if (!body.shape)
                continue;

            Color bodyColor;
            if (body.invMass == 0.0f)
            {
                bodyColor = DARKGRAY;
            }
            else
            {
                bodyColor = body.isAwake ? Color{41, 98, 255, 255} : Color{148, 163, 184, 255};
            }

            if (body.shape->type == ShapeType::Box)
            {
                const auto &box = static_cast<const Box &>(*body.shape);
                Rectangle rect = {body.position.x, body.position.y, box.width, box.height};
                DrawRectanglePro(rect, {box.width * 0.5f, box.height * 0.5f}, body.angle * RAD2DEG, bodyColor);
            }
            else if (body.shape->type == ShapeType::Circle)
            {
                const auto &circ = static_cast<const Circle &>(*body.shape);
                DrawCircleV({body.position.x, body.position.y}, circ.radius, bodyColor);

                Vec2 dir(std::cos(body.angle) * circ.radius, std::sin(body.angle) * circ.radius);
                DrawLineV({body.position.x, body.position.y}, {body.position.x + dir.x, body.position.y + dir.y}, WHITE);
            }
        }

        if (isDragging)
        {
            Vec2 mousePos(GetMouseX(), GetMouseY());
            DrawCircleV({dragStart.x, dragStart.y}, 6.0f, RED);
            DrawLineEx({dragStart.x, dragStart.y}, {mousePos.x, mousePos.y}, 2.5f, RED);
        }

        if (showBroadPhaseDebug)
        {
            for (const auto &entry : world.GetAABBEntries())
            {
                float w = entry.maxX - entry.minX;
                float h = entry.maxY - entry.minY;
                DrawRectangleLinesEx({entry.minX, entry.minY, w, h}, 1.0f, Color{34, 197, 94, 160});
            }

            for (const auto &[i, j] : world.GetBroadPhasePairs())
            {
                if (i < (int)bodies.size() && j < (int)bodies.size())
                {
                    Vec2 pA = bodies[i].position;
                    Vec2 pB = bodies[j].position;
                    DrawLineEx({pA.x, pA.y}, {pB.x, pB.y}, 1.5f, Color{239, 68, 68, 180});
                }
            }
        }
    }

    void RenderUI()
    {
        const auto &bodies = world.GetBodies();
        int totalBodies = (int)bodies.size();
        int awakeCount = 0;
        int sleepCount = 0;

        for (const auto &b : bodies)
        {
            if (b.invMass == 0.0f) continue;
            if (b.isAwake) awakeCount++;
            else sleepCount++;
        }

        DrawRectangle(12, 12, 360, 200, Fade(WHITE, 0.90f));
        DrawRectangleLines(12, 12, 360, 200, Color{203, 213, 225, 255});

        DrawFPS(24, 20);

        const char *sceneName = "1: Box Stacking";
        if (currentScene == DemoScene::Constraints) sceneName = "2: Distance Constraints";
        else if (currentScene == DemoScene::RestitutionFriction) sceneName = "3: Restitution & Friction";
        else if (currentScene == DemoScene::StressTest) sceneName = "4: Stress Test";

        DrawText(TextFormat("Scene: %s", sceneName), 24, 46, 16, DARKBLUE);
        DrawText(TextFormat("Bodies: %d  (Awake: %d | Sleep: %d)", totalBodies, awakeCount, sleepCount), 24, 68, 15, DARKGRAY);

        DrawText("Keys: [1] Stacking  [2] Joints  [3] Friction  [4] Stress", 24, 94, 13, BLACK);
        DrawText(TextFormat("Tool: [Space] Shape: %s  |  [R] Reset", (currentSpawnShape == SpawnShape::Box ? "Box" : "Circle")), 24, 116, 13, DARKGRAY);
        DrawText("Mouse: Left Click Drag to Shoot  |  Scroll to Resize", 24, 138, 13, DARKGRAY);
        DrawText(TextFormat("Debug: [B] Broadphase View: %s (Pairs: %d)", (showBroadPhaseDebug ? "ON" : "OFF"), (int)world.GetBroadPhasePairs().size()), 24, 158, 13, showBroadPhaseDebug ? MAROON : DARKGRAY);

        DrawCircle(24, 184, 6, Color{41, 98, 255, 255});
        DrawText("Awake", 36, 178, 13, DARKGRAY);
        DrawCircle(100, 184, 6, Color{148, 163, 184, 255});
        DrawText("Sleeping", 112, 178, 13, DARKGRAY);
        DrawCircle(180, 184, 6, DARKGRAY);
        DrawText("Static", 192, 178, 13, DARKGRAY);
    }
};

int main()
{
    PhysicsApp app;
    app.Run();
    return 0;
}