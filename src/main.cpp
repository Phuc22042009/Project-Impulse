#include "physics/PhysicsWorld.h"
#include "raylib.h"
PhysicsWorld world;

enum class SpawnShape
{
    Box,
    Circle
};
SpawnShape currentSpawnShape = SpawnShape::Box;
Vec2 dragStart;
bool isDragging = false;
float spawnSize = 20.0f;

void SpawnBody(Vec2 position, Vec2 velocity)
{
    if (currentSpawnShape == SpawnShape::Box)
    {
        auto boxShape = std::make_unique<Box>();
        boxShape->width = spawnSize;
        boxShape->height = spawnSize;
        RigidBody2D body(1.0f, std::move(boxShape));
        body.position = position;
        body.velocity = velocity;
        body.restitution = 0.5f;
        body.staticFriction = 0.5f;
        world.AddBody(std::move(body));
    }
    else
    {
        auto circleShape = std::make_unique<Circle>();
        circleShape->radius = spawnSize;
        circleShape->center = position;
        RigidBody2D body(1.0f, std::move(circleShape));
        body.position = position;
        body.velocity = velocity;
        body.staticFriction = 0.5f;
        body.restitution = 0.5f;
        world.AddBody(std::move(body));
    }
}
void Input()
{
    if (IsKeyPressed(KEY_B))
        currentSpawnShape = SpawnShape::Box;
    if (IsKeyPressed(KEY_C))
        currentSpawnShape = SpawnShape::Circle;

    float scroll = GetMouseWheelMove();
    if (scroll != 0)
    {
        spawnSize = std::clamp(spawnSize + scroll * 5.0f, 5.0f, 100.0f);
    }

    Vec2 mousepos = Vec2(GetMouseX(), GetMouseY());
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        isDragging = true;
        dragStart = mousepos;
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && isDragging)
    {
        Vec2 dragDelta = mousepos - dragStart;
        Vec2 velocity = dragDelta * 5.0f;
        SpawnBody(dragStart, velocity);
        isDragging = false;
    }
}

void Update(float dt)
{
    world.Step(dt);
    Input();
}
void Draw()
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    DrawFPS(400, 400);

    for (const auto &body : world.GetBodies())
    {
        if (!body.shape)
            continue;
        if (body.shape->type == ShapeType::Box)
        {
            const auto &box = static_cast<const Box &>(*body.shape);
            Color color = body.isAwake ? BLUE : GRAY;
            Rectangle rect = {
                body.position.x,
                body.position.y,
                box.width,
                box.height};
            DrawRectanglePro(rect, {box.width * 0.5f, box.height * 0.5f}, body.angle * RAD2DEG, color);
        }
        else if (body.shape->type == ShapeType::Circle)
        {
            const auto &circ = static_cast<const Circle &>(*body.shape);
            Color color = body.isAwake ? RED : GRAY;
            DrawCircle(body.position.x, body.position.y, circ.radius, color);
        }
    }
    if (isDragging)
    {
        Vec2 mousepos = Vec2(GetMouseX(), GetMouseY());
        if (currentSpawnShape == SpawnShape::Box)
        {
            DrawRectanglePro({dragStart.x, dragStart.y, spawnSize, spawnSize}, {spawnSize * 0.5f, spawnSize * 0.5f}, 0.0f, Fade(BLUE, 0.5f));
        }
        else
        {
            DrawCircleV({dragStart.x, dragStart.y}, spawnSize, Fade(RED, 0.5f));
        }
        DrawLineV({dragStart.x, dragStart.y}, {mousepos.x, mousepos.y}, BLACK);
    }
    EndDrawing();
}
int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(800, 600, "Physics Engine");
    SetTargetFPS(60);

    auto boxShape = std::make_unique<Box>();
    boxShape->width = 800.0f;
    boxShape->height = 50.0f;

    RigidBody2D floor(0.0f, std::move(boxShape));
    floor.position = Vec2(400.0f, 575.0f);
    world.AddBody(std::move(floor));
    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        Update(dt);
        Draw();
    }
    return 0;
}
