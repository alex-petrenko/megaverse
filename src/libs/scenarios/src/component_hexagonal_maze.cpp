#include <mazes/kruskal.h>
#include <mazes/honeycombmaze.h>

#include <scenarios/layout_utils.hpp>
#include <scenarios/component_hexagonal_maze.hpp>


using namespace Magnum;
using namespace VoxelWorld;


HexagonalMazeComponent::HexagonalMazeComponent(VoxelWorld::Scenario &scenario)
: ScenarioComponent{scenario}
{
}

HexagonalMazeComponent::~HexagonalMazeComponent() = default;

void HexagonalMazeComponent::reset(Env &, Env::EnvState &envState)
{
    const auto size = 11;
    maze = std::make_unique<HoneyCombMaze>(size);

    Kruskal algorithm;

    TLOG(DEBUG) << "Initialising graph...";
    maze->InitialiseGraph();
    TLOG(DEBUG) << "Generating maze...";
    maze->GenerateMaze(&algorithm);

    const auto [xmin, ymin, xmax, ymax] = maze->GetCoordinateBounds();
    xMin = xmin, yMin = ymin, xMax = xmax, yMax = ymax;

//    mazeScale = frand(envState.rng) * 0.8f + 3.0f;
    mazeScale = 3.0f;
    wallHeight = frand(envState.rng) * 2.0f + 2.0f;

    xMin *= mazeScale, xMax *= mazeScale, yMin *= mazeScale, yMax *= mazeScale;
}

void HexagonalMazeComponent::addDrawablesAndCollisions(DrawablesMap &drawables, Env::EnvState &envState) const
{
    auto scale = Magnum::Vector3{float(xMax - xMin), 1.0f, float(yMax - yMin)};
    auto translation = Magnum::Vector3{float(xMax + xMin) / 2, 0.5f, float(yMax + yMin) / 2};
    addStaticCollidingBox(drawables, envState, scale, translation, ColorRgb::LAYOUT);

    // TODO: debug stuff, remove
//    for (int x = -1; x >= -7; --x)
//        for (int z = -1; z >= -7; --z) {
//            scale = Magnum::Vector3{0.1, 1, 0.1};
//            translation = Magnum::Vector3{float(x), 1, float(z)};
//
//            addStaticCollidingBox(drawables, envState, scale, translation, ColorRgb::GREEN);
//        }
//
//    for (int x = 20; x >= -7; --x)
//        for (int z = 0; z >= 0; --z) {
//            scale = Magnum::Vector3{0.1, 1, 0.1};
//            translation = Magnum::Vector3{float(x), 1, float(z)};
//            addStaticCollidingBox(drawables, envState, scale, translation, ColorRgb::RED);
//        }

    auto adjList = maze->getAdjacencyList();
    for (auto &cellAdjList : adjList) {
        for (auto &[cell, border] : cellAdjList) {
            const auto lineBorder = dynamic_cast<LineBorder *>(border.get());
            auto [x1, z1, x2, z2] = lineBorder->getBorderCoords();
            x1 *= mazeScale, z1 *= mazeScale, x2 *= mazeScale, z2 *= mazeScale;

            const auto length = 0.5f * sqrtf(float(sqr(x1 - x2) + sqr(z1 - z2)));
            const Vector3 wallScale{length, wallHeight, 0.15f};
            const Vector3 wallTranslation{float(x1 + x2) / 2, 1, float(z1 + z2) / 2};

            const auto deltaX = x1 - x2;
            const auto deltaZ = z1 - z2;

            float rotationY = M_PI_2;
            if (std::fabs(deltaX) > EPSILON) {
                const auto tanAlpha = deltaZ / deltaX;
                rotationY = -atanf(tanAlpha);
            }

            auto &layoutBox = envState.scene->addChild<Object3D>();
            layoutBox.scale(wallScale).rotateY(Rad(rotationY)).translate(wallTranslation);
            drawables[DrawableType::Box].emplace_back(&layoutBox, rgb(ColorRgb::DARK_BLUE));

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});
            auto &collisionBox = layoutBox.addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
            collisionBox.syncPose();
            envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));
        }
    }
}
