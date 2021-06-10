#include <mazes/kruskal.h>
#include <mazes/honeycombmaze.h>

#include <scenarios/layout_utils.hpp>
#include <scenarios/component_hexagonal_maze.hpp>


using namespace Magnum;
using namespace Megaverse;


HexagonalMazeComponent::HexagonalMazeComponent(Megaverse::Scenario &scenario)
: ScenarioComponent{scenario}
{
}

HexagonalMazeComponent::~HexagonalMazeComponent() = default;

void HexagonalMazeComponent::reset(Env &, Env::EnvState &envState)
{
    mazeSize = randRange(minSize, maxSize, envState.rng);
    maze = std::make_unique<HoneyCombMaze>(mazeSize);

    Kruskal algorithm;

    TLOG(DEBUG) << "Initialising graph...";
    maze->InitialiseGraph();
    TLOG(DEBUG) << "Generating maze...";
    maze->GenerateMaze(&algorithm);

    const auto [xmin, ymin, xmax, ymax] = maze->GetCoordinateBounds();
    xMin = xmin, yMin = ymin, xMax = xmax, yMax = ymax;

    mazeScale = 3.5f;
    wallHeight = frand(envState.rng) * 0.55f + 0.85f;
    omitWallsProbability = frand(envState.rng) * (omitWallsProbabilityMax - omitWallsProbabilityMin) + omitWallsProbabilityMin;
    wallLandmarkProbability = frand(envState.rng) * 0.15f + 0.15f;

    bottomEdgingColor = sampleRandomColor(envState.rng);
    topEdgingColor = sampleRandomColor(envState.rng);

    xMin *= mazeScale, xMax *= mazeScale, yMin *= mazeScale, yMax *= mazeScale;
}

void HexagonalMazeComponent::addDrawablesAndCollisions(DrawablesMap &drawables, Env::EnvState &envState) const
{
    auto scale = Magnum::Vector3{float(xMax - xMin), 0.0001f, float(yMax - yMin)};
    auto translation = Magnum::Vector3{float(xMax + xMin) / 2, 0.0f, float(yMax + yMin) / 2};
    addStaticCollidingBox(drawables, envState, scale, translation, randomLayoutColor(envState.rng));

    std::set<std::pair<int, int>> existingWalls;

    auto adjList = maze->getAdjacencyList();
    for (auto cellIdx = 0; cellIdx < int(adjList.size()); ++cellIdx) {
        auto &cellAdjList = adjList[cellIdx];

        for (auto &[adjCellIdx, border] : cellAdjList) {
            auto cellPair = std::make_pair(cellIdx, adjCellIdx);
            if (cellPair.first > cellPair.second)
                std::swap(cellPair.first, cellPair.second);  // sort the pair

            // check if this is not an outside border
            if (adjCellIdx != -1) {
                if (existingWalls.count(cellPair)) continue;

                if (frand(envState.rng) < omitWallsProbability) {
                    // remove this particular wall
                    continue;
                }
            }

            existingWalls.insert(cellPair);

            const auto lineBorder = dynamic_cast<LineBorder *>(border.get());
            auto [x1, z1, x2, z2] = lineBorder->getBorderCoords();
            x1 *= mazeScale, z1 *= mazeScale, x2 *= mazeScale, z2 *= mazeScale;

            const auto length = 0.5f * sqrtf(float(sqr(x1 - x2) + sqr(z1 - z2)));
            const Vector3 wallScale{length, wallHeight, 0.15f};
            const Vector3 wallTranslation{float(x1 + x2) / 2, wallHeight, float(z1 + z2) / 2};

            const auto deltaX = x1 - x2;
            const auto deltaZ = z1 - z2;

            float rotationY = M_PI_2;
            if (std::fabs(deltaX) > EPSILON) {
                const auto tanAlpha = deltaZ / deltaX;
                rotationY = -atanf(tanAlpha);
            }

            auto &layoutBox = envState.scene->addChild<Object3D>();
            if (frand(envState.rng) < wallLandmarkProbability) {
                const auto landmarkWidth = 0.15f, landmarkHeight = landmarkWidth * length / wallHeight;

                int numLandmarks = randRange(2, 5, envState.rng);
                for (int li = 0; li < numLandmarks; ++li) {
                    const Vector3 landmarkScale{landmarkWidth, landmarkHeight, frand(envState.rng) * 1.2f + 1.5f};
                    auto &landmarkBox = layoutBox.addChild<Object3D>();
                    const Vector3 landmarkTranslation{float(li % 2 == 1) * landmarkWidth * 2, float(li > 1) * landmarkHeight * 2 - 0.2f, 0};
                    landmarkBox.scaleLocal(landmarkScale).translate(landmarkTranslation);
                    drawables[DrawableType::Box].emplace_back(&landmarkBox, rgb(sampleRandomColor(envState.rng)));
                }
            }

            layoutBox.scale(wallScale).rotateY(Rad(rotationY)).translate(wallTranslation);
            drawables[DrawableType::Box].emplace_back(&layoutBox, rgb(ColorRgb::DARK_BLUE));

            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});
            auto &collisionBox = layoutBox.addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics->bWorld);
            collisionBox.syncPose();
            envState.physics->collisionShapes.emplace_back(std::move(bBoxShape));

            // top and bottom edging
            {
                auto &bottomEdgingBox = envState.scene->addChild<Object3D>();
                const Vector3 edgingScale{length * 1.02f, wallHeight * 0.12f, 0.2f};
                const Vector3 bottomEdgingTranslation{wallTranslation.x(), edgingScale.y(), wallTranslation.z()};
                bottomEdgingBox.scale(edgingScale).rotateY(Rad(rotationY)).translate(bottomEdgingTranslation);
                drawables[DrawableType::Box].emplace_back(&bottomEdgingBox, rgb(bottomEdgingColor));

//                auto &topEdgingBox = envState.scene->addChild<Object3D>();
//                const Vector3 topEdgingTranslation{wallTranslation.x(), wallHeight * 2, wallTranslation.z()};
//                topEdgingBox.scale(edgingScale).rotateY(Rad(rotationY)).translate(topEdgingTranslation);
//                drawables[DrawableType::Box].emplace_back(&topEdgingBox, rgb(topEdgingColor));
            }
        }
    }
}
