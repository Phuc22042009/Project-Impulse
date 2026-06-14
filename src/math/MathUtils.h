#pragma once
#include <cmath>
#include <algorithm>
#if defined(PI)
    #undef PI
#endif

#if defined(DEG2RAD)
    #undef DEG2RAD
#endif
constexpr float PI = 3.14159265358979f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float MATH_RAD2DEG = 180.0f/ PI;


