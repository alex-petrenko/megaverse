#include <regex>
#include <iomanip>

#include <util/string_utils.hpp>
#include <util/filesystem_utils.hpp>

#include <scenarios/scenario_sokoban.hpp>


using namespace VoxelWorld;
using namespace Magnum::Math::Literals;


namespace
{

enum SokobanNotation
{
    EMPTY_CELL = ' ',
    GOAL_CELL = '.',
    PLAYER_CELL = '@',
    PLAYER_ON_GOAL = '+',
    BOX_CELL = '$',
    BOX_ON_GOAL = '*',
    WALL_CELL = '#',
};

enum SokobanTerrain
{
    SOKO_EMPTY = 0,
    SOKO_WALL = 1,
    SOKO_GOAL = 2,
};

}


SokobanScenario::SokobanScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this, 100, 0, 0, 0, 2}
{
    auto envvarBoxobanPath = std::getenv("BOXOBAN_LEVELS");
    if (!envvarBoxobanPath || strlen(envvarBoxobanPath) == 0)
        TLOG(DEBUG) << "Could not find Boxoban levels through the environment variable 'BOXOBAN_LEVELS'";
    else
        boxobanLevelsDir = envvarBoxobanPath;

    constexpr auto backupLocation = "~/datasets/boxoban";

    if (boxobanLevelsDir.empty())
        boxobanLevelsDir = backupLocation;

    if (contains(boxobanLevelsDir, '~')) {
        auto envvarHome = std::getenv("HOME");
        if (!envvarHome || strlen(envvarHome) == 0)
            TLOG(FATAL) << "Could not query HOME env var to resolve ~ in the path to Boxoban levels dir";

        boxobanLevelsDir = std::regex_replace(boxobanLevelsDir, std::regex("~"), envvarHome);
    }

    auto dirWithLevels = pathJoin(boxobanLevelsDir, levelSet, levelSplit);

    for (int levelFileIdx = 0; levelFileIdx <= 999; ++levelFileIdx) {
        std::ostringstream ss;
        ss << std::setw(3) << std::setfill('0') << levelFileIdx << ".txt";
        auto levelFilePath = pathJoin(dirWithLevels, ss.str());
        if (fileExists(levelFilePath))
            allSokobanLevelFiles.emplace_back(levelFilePath);
    }

    if (allSokobanLevelFiles.empty())
        TLOG(FATAL) << "Could not find any Boxoban levels. Set envvar BOXOBAN_LEVELS or put unzipped Boxoban folder "
                       "(named boxoban) containing unfiltered/medium/hard level splits into ~/datasets";

    TLOG(INFO) << allSokobanLevelFiles.size() << " boxoban level files found";
}

SokobanScenario::~SokobanScenario() = default;

void SokobanScenario::reloadLevels()
{
    const auto levelFilePath = randomSample(allSokobanLevelFiles, envState.rng);

    std::vector<char> buffer;
    const auto bytesRead = readAllBytes(levelFilePath, buffer);
    if (!bytesRead)
        TLOG(FATAL) << "Could not read the level file " << levelFilePath;

    std::string content{buffer.begin(), buffer.end()};
    auto lines = splitString(content, "\n");
    SokobanLevel level;
    for (int i = 0; i < int(lines.size()); ++i) {
        if (startsWith(lines[i], ";")) {
            if (i > 0) levels.emplace_back(std::move(level));
            level = SokobanLevel{};
        } else
            level.rows.emplace_back(lines[i]);
    }

    std::shuffle(levels.begin(), levels.end(), envState.rng);
}

void SokobanScenario::reset()
{
    vg.reset(env, envState);
    solved = false;
    agentPositions.clear(), boxesCoords.clear();
    length = width = 0;
    numBoxes = numBoxesOnGoal = 0;

    if (levels.empty())
        reloadLevels();

    currLevel = levels.back();
    levels.pop_back();

    createLayout();
}

void SokobanScenario::createLayout()
{
    constexpr int wallHeight = 2;
    auto &g = vg.grid;

    static const std::vector<ColorRgb> floorColors = {
        ColorRgb::LAYOUT_DEFAULT,
        ColorRgb::VERY_LIGHT_YELLOW,
        ColorRgb::VERY_LIGHT_BLUE,
        ColorRgb::VERY_LIGHT_ORANGE,
        ColorRgb::DARK_GREY,
    };
    auto floorColor = randomSample(floorColors, envState.rng);

    length = int(currLevel.rows.size());
    for (int x = 0; x < length; ++x) {
        const auto &row = currLevel.rows[x];
        width = std::max(width, int(row.size()));

        for (int z = 0; z < int(row.size()); ++z) {
            g.set({x, 0, z}, makeVoxel<VoxelWithPhysicsObjects>(VOXEL_SOLID | VOXEL_OPAQUE, TERRAIN_NONE, floorColor));

            if (row[z] == WALL_CELL) {
                for (int y = 1; y <= wallHeight; ++y)
                    g.set({x, y, z}, makeVoxel<VoxelWithPhysicsObjects>(VOXEL_SOLID));

                g.get(VoxelCoords{x, 1, z})->terrain = SOKO_WALL;
            }

            if (row[z] == PLAYER_CELL || row[z] == PLAYER_ON_GOAL) {
                for (int agentIdx = 0; agentIdx < env.getNumAgents(); ++agentIdx) {
                    const float agentX = float(x) + float(agentIdx % 2) * 0.5f;
                    const float agentZ = float(z) + float(agentIdx % 4 > 1) * 0.5f;
                    agentPositions.emplace_back(agentX * voxelSize, voxelSize + 0.3 * float(agentIdx) * voxelSize, agentZ * voxelSize);
                }
            }

            if (row[z] == GOAL_CELL || row[z] == PLAYER_ON_GOAL)
                g.set({x, 1, z}, makeVoxel<VoxelWithPhysicsObjects>(VOXEL_EMPTY, SOKO_GOAL));

            if (row[z] == BOX_CELL || row[z] == BOX_ON_GOAL) {
                boxesCoords.emplace_back(x, 1, z);
                ++numBoxes;
            }
        }
    }
}

void SokobanScenario::step()
{
    // TODO: in multi-agent envs they can push each other and thus move unmovable boxes. Fix

    // moving the boxes logic
    for (int i = 0; i < env.getNumAgents(); ++i) {
        const auto a = envState.currAction[i];
        auto &agent = envState.agents[i];

        if (!!(a & Action::Interact)) {
            auto t = agent->interactLocation()->absoluteTransformation().translation();
            auto voxel = vg.grid.getWithVector(t);

            if (voxel && voxel->physicsObject) {
                const auto boxPos = vg.grid.getCoords(t);
                const auto agentPos = vg.grid.getCoords(agent->absoluteTransformation().translation());
                const auto dist = manhattanDistance(agentPos, boxPos);

                if (dist == 1) {
                    // only move the box if we are in the adjacent cell
                    auto deltaPos = boxPos - agentPos;
                    auto desiredPos = boxPos + deltaPos;

                    bool occupied = false;
                    for (int j = 0; j < env.getNumAgents(); ++j)
                        if (vg.grid.getCoords(envState.agents[j]->absoluteTransformation().translation()) == desiredPos) {
                            occupied = true;
                            break;
                        }

                    if (!occupied) {
                        if (!vg.grid.hasVoxel(desiredPos))
                            vg.grid.set(desiredPos, makeVoxel<VoxelWithPhysicsObjects>(VOXEL_EMPTY));

                        auto desiredPosVoxel = vg.grid.get(desiredPos);
                        if (desiredPosVoxel->terrain != SOKO_WALL && !desiredPosVoxel->physicsObject) {
                            desiredPosVoxel->physicsObject = voxel->physicsObject;
                            desiredPosVoxel->physicsObject->parent()->translate(Magnum::Vector3{deltaPos} * voxelSize);
                            desiredPosVoxel->physicsObject->syncPose();
                            voxel->physicsObject = nullptr;

                            // moved the box
                            if (voxel->terrain != SOKO_GOAL && desiredPosVoxel->terrain == SOKO_GOAL) {
                                // moved the box to the goal position
                                ++numBoxesOnGoal;
                                rewardTeam(Str::sokobanBoxOnTarget, i, 1);

                                if (numBoxesOnGoal == numBoxes && !solved) {
                                    TLOG(INFO) << "Done!";
                                    solved = true;
                                    rewardTeam(Str::sokobanAllBoxesOnTarget, i, 1);
                                    doneWithTimer();
                                }

                            } else if (voxel->terrain == SOKO_GOAL && desiredPosVoxel->terrain != SOKO_GOAL) {
                                // moved the box from the goal position - penalty
                                --numBoxesOnGoal;
                                rewardTeam(Str::sokobanBoxLeavesTarget, i, 1);
                            }
                        }
                    }
                }
            }
        }
    }
}

std::vector<Magnum::Vector3> SokobanScenario::agentStartingPositions()
{
    return agentPositions;
}

void SokobanScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    addDrawablesAndCollisionObjectsFromVoxelGrid(vg, drawables, envState, vg.grid.getVoxelSize());

    const static std::map<SokobanTerrain, ColorRgb> colors = {
        {SOKO_WALL, ColorRgb::LIGHT_ORANGE},
        {SOKO_GOAL, ColorRgb::LIGHT_GREEN},
    };

    const static std::map<SokobanTerrain, float> height = {
        {SOKO_WALL, 0.35f},
        {SOKO_GOAL, 0.025f},
    };

    auto &g = vg.grid;
    for (int x = 0; x < length; ++x) {
        for (int z = 0; z < width; ++z) {
            if (!g.hasVoxel({x, 1, z}))
                continue;
            auto v = g.get({x, 1, z});
            if (v->terrain == SOKO_EMPTY)
                continue;

            const auto pos = Magnum::Vector3(voxelSize * float(x) + voxelSize / 2, voxelSize, voxelSize * float(z) + voxelSize / 2);
            const auto h = height.at(SokobanTerrain(v->terrain));

            auto &terrainObject = envState.scene->addChild<Object3D>(envState.scene.get());
            terrainObject.scale({1, h, 1});
            terrainObject.translate({0.0, h, 0.0});
            terrainObject.translate(pos);

            drawables[DrawableType::Box].emplace_back(&terrainObject, rgb(colors.at(SokobanTerrain(v->terrain))));
        }
    }

    for (auto box : boxesCoords) {
        auto scale = Magnum::Vector3{voxelSize / 2, 0.45f, voxelSize / 2} * 0.8f;
        auto translation = Magnum::Vector3{float(box.x()) + 0.5f, float(box.y()) + 0.2f, float(box.z()) + 0.5f} * voxelSize;

        auto &layoutBox = envState.scene->addChild<Object3D>();
        layoutBox.scale(scale).translate(translation);
        drawables[DrawableType::Box].emplace_back(&layoutBox, rgb(ColorRgb::DARK_BLUE));

        auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});
        auto &collisionBox = layoutBox.addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics->bWorld);
        collisionBox.setCollisionScale({1.15, 3, 1.15});
        collisionBox.setCollisionOffset({0, 0.6, 0});
        collisionBox.syncPose();
        envState.physics->collisionShapes.emplace_back(std::move(bBoxShape));

        if (!g.hasVoxel({box}))
            g.set(box, makeVoxel<VoxelWithPhysicsObjects>(VOXEL_EMPTY));

        g.get(box)->physicsObject = &collisionBox;
    }
}
