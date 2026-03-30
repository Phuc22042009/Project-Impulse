#include <iostream>
#include "physics/Shape.h"
#include "raylib.h"

void DrawShape(const Shape& shape, Vec2 pos, float angle, Color color) {
    int n = (int)shape.verticles.size();
    for (int i = 0; i< n; i++) {
        Vec2 a = pos + shape.verticles[i].Rotated(angle);
        Vec2 b = pos + shape.verticles[(i + 1) % n].Rotated(angle);
        DrawLine((int)a.x, (int)a.y, (int)b.x, (int)b.y, color);
    }
}
int main() {
     // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "Physics Engine");

    SetTargetFPS(60);               

    Shape box = Shape::MakeBox(60,40);
    Shape triangle = Shape::MakePolygon(3, 50);

    float angle = 0.0f;
    Vec2 pos = Vec2(320, 300);
    while (!WindowShouldClose())
    {
        angle += 0.05f;
        BeginDrawing();

            ClearBackground({20, 22, 28, 255});

            DrawShape(box, pos, angle, BLUE);
            DrawShape(triangle, Vec2(200,300), angle, GREEN);

        EndDrawing();
    }
    CloseWindow();
}
