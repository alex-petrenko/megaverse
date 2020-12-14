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

void HexagonalMazeComponent::reset(Env &, Env::EnvState &)
{
    const auto size = 5;
    maze = std::make_unique<HoneyCombMaze>(size);

    Kruskal algorithm;

    TLOG(DEBUG) << "Initialising graph...";
    maze->InitialiseGraph();
    TLOG(DEBUG) << "Generating maze...";
    maze->GenerateMaze(&algorithm);

    const auto [xmin, ymin, xmax, ymax] = maze->GetCoordinateBounds();
    xMin = xmin, yMin = ymin, xMax = xmax, yMax = ymax;
}

void HexagonalMazeComponent::addDrawablesAndCollisions(DrawablesMap &drawables, Env::EnvState &envState) const
{
    auto scale = Magnum::Vector3{float(xMax - xMin), 1.0f, float(yMax - yMin)};
    auto translation = Magnum::Vector3{float(xMax + xMin) / 2, 0.5f, float(yMax + yMin) / 2};
    addStaticCollidingBox(drawables, envState, scale, translation, scale, ColorRgb::LAYOUT);

    auto adjList = maze->getAdjacencyList();
    for (auto &cellAdjList : adjList) {
        for (auto &[cell, border] : cellAdjList) {
            const auto lineBorder = dynamic_cast<LineBorder *>(border.get());
            const auto [x1, z1, x2, z2] = lineBorder->getBorderCoords();

            const auto length = sqrtf(float(sqr(x1 - x2) + sqr(z1 - z2)));
            const Vector3 wallScale{length, 3.0f, 0.1f};
            const Vector3 wallTranslation{float(x1 + x2) / 2, 1, float(z1 + z2)};

            const auto deltaX = x1 - x2;
            const auto deltaZ = z1 - z2;
            const auto rotationY = std::atan2(deltaZ, deltaX);

            auto &layoutBox = envState.scene->addChild<Object3D>();
            layoutBox.scale(wallScale).rotateY(Rad(rotationY)).translate(wallTranslation);
            drawables[DrawableType::Box].emplace_back(&layoutBox, rgb(ColorRgb::DARK_BLUE));

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{wallScale.x(), wallScale.y(), wallScale.z()});
            auto &collisionBox = layoutBox.addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
            collisionBox.syncPose();
            envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));
        }
    }
}
