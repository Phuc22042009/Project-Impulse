#ifdef NDEBUG
#undef NDEBUG
#endif
#include "physics/PhysicsWorld.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include <memory>

bool NearEqual(float a, float b, float eps = 1e-4f)
{
    return std::fabs(a - b) <= eps;
}

void TestVec2_LengthAndNormalize()
{
    Vec2 v(6.0f, 9.0f);

    assert(NearEqual(v.LenSq(), 117.0f));
    assert(NearEqual(v.Len(), std::sqrt(117.0f)));

    Vec2 unit = v.Normalized();
    assert(NearEqual(unit.x, 6.0f / std::sqrt(117.0f)));
    assert(NearEqual(unit.y, 9.0f / std::sqrt(117.0f)));
    assert(NearEqual(unit.Len(), 1.0f));

    std::cout << "Vec2 math tests passed." << std::endl;
}

void TestVec2_ZeroNormalize()
{
    Vec2 v2(0.0f, 0.0f);
    Vec2 unit = v2.Normalized();
    assert(NearEqual(unit.x, 0.0f));
    assert(NearEqual(unit.y, 0.0f));
    assert(NearEqual(unit.Len(), 0.0f));

    std::cout << "Vec2 zero normalization test passed." << std::endl;
}

void TestVec2_DotAndCrossProduct()
{
    Vec2 v1(3.0f, 4.0f);
    Vec2 v2(5.0f, 6.0f);

    float dot = DotProduct(v1, v2);
    assert(NearEqual(dot, 3.0f * 5.0f + 4.0f * 6.0f));

    float cross = CrossProduct(v1, v2);
    assert(NearEqual(cross, 3.0f * 6.0f - 4.0f * 5.0f));

    std::cout << "Vec2 dot and cross product tests passed." << std::endl;
}

void TestVec2_Rotate90DegreesAndPerp()
{
    Vec2 v(1.0f, 0.0f);
    Vec2 rotated = v.Rotated(PI * 0.5f);
    Vec2 perp = v.Perp();

    assert(NearEqual(rotated.x, 0.0f));
    assert(NearEqual(rotated.y, 1.0f));
    assert(NearEqual(perp.x, 0.0f));
    assert(NearEqual(perp.y, 1.0f));

    std::cout << "Vec2 rotation and perpendicular tests passed." << std::endl;
}

void TestBroadPhase_OverlappingBodies_GeneratesCandidatePair()
{
    PhysicsWorld world;
    auto boxA = std::make_unique<Box>(50.0f, 50.0f);

    RigidBody2D bodyA(1.0f, std::move(boxA));
    bodyA.position = Vec2(100.0f, 100.0f);
    world.AddBody(std::move(bodyA));

    auto boxB = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyB(1.0f, std::move(boxB));
    bodyB.position = Vec2(120.0f, 120.0f);
    world.AddBody(std::move(bodyB));

    world.Step(1.0f / 60.0f);

    const auto &pairs = world.GetBroadPhasePairs();
    assert(pairs.size() == 1);
    assert((pairs[0].first == 0 && pairs[0].second == 1) || (pairs[0].first == 1 && pairs[0].second == 0));

    std::cout << "Broad phase overlapping bodies test passed." << std::endl;
}

void TestBroadPhase_SeparatedAlongX_ProducesZeroPairs()
{
    PhysicsWorld world;
    auto boxA = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyA(1.0f, std::move(boxA));
    bodyA.position = Vec2(100.0f, 100.0f);
    world.AddBody(std::move(bodyA));

    auto boxB = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyB(1.0f, std::move(boxB));
    bodyB.position = Vec2(200.0f, 100.0f);
    world.AddBody(std::move(bodyB));

    world.Step(1.0f / 60.0f);

    const auto &pairs = world.GetBroadPhasePairs();
    assert(pairs.empty());

    std::cout << "Broad phase separated bodies test passed." << std::endl;
}

void TestBroadPhase_OverlapXButSeparatedY_IsPrunedCorrectly()
{
    PhysicsWorld world;
    auto boxA = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyA(1.0f, std::move(boxA));
    bodyA.position = Vec2(100.0f, 100.0f);
    world.AddBody(std::move(bodyA));

    auto boxB = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyB(1.0f, std::move(boxB));
    bodyB.position = Vec2(120.0f, 200.0f);
    world.AddBody(std::move(bodyB));

    world.Step(1.0f / 60.0f);

    const auto &pairs = world.GetBroadPhasePairs();
    assert(pairs.empty());

    std::cout << "Broad phase overlapping along X but separated along Y test passed." << std::endl;
}

void TestBroadPhase_MultipleBodies_ScalesWithoutDuplicatePairs()
{
    PhysicsWorld world;
    const int numBodies = 10;
    for (int i = 0; i < numBodies; ++i)
    {
        auto box = std::make_unique<Box>(50.0f, 50.0f);
        RigidBody2D body(1.0f, std::move(box));
        body.position = Vec2(100.0f + i * 10.0f, 100.0f);
        world.AddBody(std::move(body));
    }

    world.Step(1.0f / 60.0f);

    const auto &pairs = world.GetBroadPhasePairs();
    assert(!pairs.empty());

    std::set<std::pair<int, int>> uniquePairs;
    for (const auto &p : pairs)
    {
        int a = std::min(p.first, p.second);
        int b = std::max(p.first, p.second);
        uniquePairs.insert({a, b});
    }
    assert(uniquePairs.size() == pairs.size());

    std::cout << "Broad phase multiple bodies scaling test passed." << std::endl;
}

void TestCircleVsCircle_Overlapping_ComputesExactPenetration()
{
    auto circleA = std::make_unique<Circle>(30.0f);
    RigidBody2D bodyA(1.0f, std::move(circleA));
    bodyA.position = Vec2(100.0f, 100.0f);

    auto circleB = std::make_unique<Circle>(30.0f);
    RigidBody2D bodyB(1.0f, std::move(circleB));
    bodyB.position = Vec2(120.0f, 100.0f);

    Manifold manifold = GetManifold(&bodyA, &bodyB);

    assert(manifold.colliding);
    assert(manifold.contactCount == 1);
    assert(NearEqual(manifold.penetration, 40.0f));

    std::cout << "Circle vs Circle overlapping penetration test passed." << std::endl;
}

void TestOBBVsOBB_SeparatedAt45Degrees_ReturnsNoCollision()
{
    auto boxA = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyA(1.0f, std::move(boxA));
    bodyA.position = Vec2(100.0f, 100.0f);
    bodyA.angle = PI / 4.0f;

    auto boxB = std::make_unique<Box>(50.0f, 50.0f);
    RigidBody2D bodyB(1.0f, std::move(boxB));
    bodyB.position = Vec2(150.0f, 150.0f);

    Manifold manifold = GetManifold(&bodyA, &bodyB);
    assert(!manifold.colliding);

    std::cout << "OBB vs OBB separated at 45 degrees test passed." << std::endl;
}

void TestOBBVsOBB_FlatStack_GeneratesTwoContactPoints()
{
    auto boxA = std::make_unique<Box>(100.0f, 20.0f);
    RigidBody2D bodyA(1.0f, std::move(boxA));
    bodyA.position = Vec2(200.0f, 200.0f);

    auto boxB = std::make_unique<Box>(100.0f, 20.0f);
    RigidBody2D bodyB(1.0f, std::move(boxB));
    bodyB.position = Vec2(200.0f, 210.0f);

    Manifold manifold = GetManifold(&bodyA, &bodyB);

    assert(manifold.colliding);
    assert(manifold.contactCount == 2);

    std::cout << "OBB vs OBB flat stack contact points test passed." << std::endl;
}

void TestOBBVsCircle_OverlappingEdge_ComputesValidManifold()
{
    auto box = std::make_unique<Box>(100.0f, 20.0f);
    RigidBody2D bodyA(1.0f, std::move(box));
    bodyA.position = Vec2(300.0f, 300.0f);

    auto circle = std::make_unique<Circle>(15.0f);
    RigidBody2D bodyB(1.0f, std::move(circle));
    bodyB.position = Vec2(310.0f, 290.0f);

    Manifold manifold = GetManifold(&bodyA, &bodyB);

    assert(manifold.colliding);
    assert(manifold.contactCount > 0);
    assert(manifold.penetration > 0.0f);

    std::cout << "OBB vs Circle overlapping edge manifold test passed." << std::endl;
}

void TestPhysicsWorld_FreeFall_FollowsSemiImplicitEuler()
{
    PhysicsWorld world;
    auto box = std::make_unique<Box>(20.0f, 20.0f);
    RigidBody2D body(1.0f, std::move(box));
    body.position = Vec2(400.0f, 100.0f);
    world.AddBody(std::move(body));

    world.Step(1.0f / 60.0f);

    float subDt = (1.0f / 60.0f) * 0.5f;
    float damp = 1.0f / (1.0f + 0.1f * subDt);
    float g = world.GetGravity().y;
    float v1 = (g * subDt) * damp;
    float y1 = 100.0f + v1 * subDt;
    float v2 = (v1 + g * subDt) * damp;
    float y2 = y1 + v2 * subDt;

    const auto &bodies = world.GetBodies();
    assert(!bodies.empty());
    const RigidBody2D &finalBody = bodies[0];

    assert(NearEqual(finalBody.velocity.y, v2));
    assert(NearEqual(finalBody.position.y, y2));

    std::cout << "PhysicsWorld free fall semi-implicit Euler test passed." << std::endl;
}

void TestPhysicsWorld_StaticBody_NeverMovesUnderImpact()
{
    PhysicsWorld world;
    auto staticBox = std::make_unique<Box>(800.0f, 50.0f);
    RigidBody2D staticBody(0.0f, std::move(staticBox));
    staticBody.position = Vec2(400.0f, 440.0f);
    world.AddBody(std::move(staticBody));

    auto box = std::make_unique<Box>(20.0f, 20.0f);
    RigidBody2D body(1.0f, std::move(box));
    body.position = Vec2(500.0f, 400.0f);
    body.velocity = Vec2(0.0f, 100.0f);
    world.AddBody(std::move(body));

    float dt = 1.0f / 60.0f;
    
    for (int i = 0; i < 60; ++i)
    {
        world.Step(dt);
    }

    const auto &bodies = world.GetBodies();
    assert(bodies.size() == 2);

    const RigidBody2D &staticFinal = bodies[0];
    assert(NearEqual(staticFinal.position.x, 400.0f));
    assert(NearEqual(staticFinal.position.y, 440.0f));

    std::cout << "PhysicsWorld static body test passed." << std::endl;
}

void TestPhysicsWorld_RestitutionOne_BouncesUpward()
{
    PhysicsWorld world;
    auto floor = std::make_unique<Box>(800.0f, 20.0f);
    RigidBody2D floorBody(0.0f, std::move(floor));
    floorBody.restitution = 1.0f;
    floorBody.position = Vec2(400.0f, 500.0f);
    world.AddBody(std::move(floorBody));

    auto box = std::make_unique<Box>(20.0f, 20.0f);
    RigidBody2D body(1.0f, std::move(box));
    body.position = Vec2(600.0f, 478.0f);
    body.velocity = Vec2(0.0f, 300.0f);
    body.restitution = 1.0f;
    world.AddBody(std::move(body));

    for (int i = 0; i < 5; ++i)
    {
        world.Step(1.0f / 60.0f);
    }

    const auto &bodies = world.GetBodies();
    assert(bodies.size() == 2);
    const RigidBody2D &finalBody = bodies[1];
    assert(finalBody.velocity.y < -200.0f);
    assert(finalBody.position.y < 478.0f);

    std::cout << "PhysicsWorld restitution test passed." << std::endl;
}

void TestPhysicsWorld_BoxStacking_RemainsStableAndSleeps()
{
    PhysicsWorld world;
    auto floor = std::make_unique<Box>(800.0f, 20.0f);
    RigidBody2D floorBody(0.0f, std::move(floor));
    floorBody.position = Vec2(450.0f, 535.0f);
    world.AddBody(std::move(floorBody));

    for (int i = 0; i < 5; i++)
    {
        auto box = std::make_unique<Box>(50.0f, 50.0f);
        RigidBody2D body(1.0f, std::move(box));
        body.position = Vec2(450.0f, 500.0f - i * (50.0f + 1.0f));
        body.restitution = 0.0f;
        body.staticFriction = 0.6f;
        world.AddBody(std::move(body));
    }

    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i)
    {
        world.Step(dt);
    }

    const auto &bodies = world.GetBodies();
    assert(bodies.size() == 6);

    for (const auto &body : bodies)
    {
        assert(body.isAwake == false);
        assert(NearEqual(body.velocity.Len(), 0.0f));
        assert(NearEqual(body.angularVelocity, 0.0f));
    }

    std::cout << "PhysicsWorld box stacking stability and sleep test passed." << std::endl;
}

int main()
{
    TestVec2_LengthAndNormalize();
    TestVec2_ZeroNormalize();
    TestVec2_DotAndCrossProduct();
    TestVec2_Rotate90DegreesAndPerp();

    TestBroadPhase_OverlappingBodies_GeneratesCandidatePair();
    TestBroadPhase_SeparatedAlongX_ProducesZeroPairs();
    TestBroadPhase_OverlapXButSeparatedY_IsPrunedCorrectly();
    TestBroadPhase_MultipleBodies_ScalesWithoutDuplicatePairs();

    TestCircleVsCircle_Overlapping_ComputesExactPenetration();
    TestOBBVsOBB_SeparatedAt45Degrees_ReturnsNoCollision();
    TestOBBVsOBB_FlatStack_GeneratesTwoContactPoints();
    TestOBBVsCircle_OverlappingEdge_ComputesValidManifold();

    TestPhysicsWorld_FreeFall_FollowsSemiImplicitEuler();
    TestPhysicsWorld_StaticBody_NeverMovesUnderImpact();
    TestPhysicsWorld_RestitutionOne_BouncesUpward();
    TestPhysicsWorld_BoxStacking_RemainsStableAndSleeps();

    std::cout << "All physics tests passed successfully!" << std::endl;
    return 0;
}