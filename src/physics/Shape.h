#pragma once
#include "../math/Vec2.h"
#include "Math.h"
#include "../math/MathUtils.h"
#include "vector"
enum ShapeType {Polygon};
struct Shape
{
    ShapeType type;
    std::vector<Vec2> verticles;
    std::vector<Vec2> normals;
    
    static Shape MakeBox(float hw, float hh) 
    {
        Shape s;
        s.type = ShapeType::Polygon;
        s.verticles = {
            {-hw, -hh},
            {hw, -hh},
            {hw, hh},
            {-hw, hh}
        };
        s.ComputeNormals();
        return s;
    }
    static Shape MakePolygon(int sides, float r) {
        Shape s;
        s.type = ShapeType::Polygon;
        for (int i = 0; i < sides; i++) {
            float angle = 2 * PI * i / sides - PI / 2;
            s.verticles.push_back(Vec2(r * cos(angle), r * sin(angle)));
        }
        s.ComputeNormals();
        return s;
    }
    private:
    void ComputeNormals() {
        normals.clear();
        int n = (int)verticles.size();
        for (int i = 0; i < n; i++) {
            Vec2 edge = verticles[(i + 1) % n] - verticles[i];

            normals.push_back(edge.Perp().Normalized());
        }
    }
};
