#include <set>

#include <Magnum/Math/Angle.h>
#include <Magnum/Math/Matrix4.h>

#include <scenarios/scenario_obstacles.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;

using namespace VoxelWorld;


namespace
{

enum {
    WALLS_SOUTH = 1,
    WALLS_NORTH = 1 << 1,
    WALLS_WEST = 1 << 2,
    WALLS_EAST = 1 << 3,
};


struct MagnumAABB
{
    MagnumAABB(Object3D &parent, const BoundingBox &bb)
    {
        min = &parent.addChild<Object3D>();
        min->translateLocal(Vector3(bb.min));

        max = &parent.addChild<Object3D>();
        max->translateLocal(Vector3(bb.max));
    }

    BoundingBox boundingBox() const
    {
        BoundingBox bb;
        bb.min = Magnum::Math::lround(min->absoluteTransformation().translation());
        bb.max = Magnum::Math::lround(max->absoluteTransformation().translation());
        bb.sort();
        return bb;
    }

public:
    Object3D *min, *max;
};


class Platform
{
public:
    explicit Platform(Object3D *parent, Rng &rng, int walls)
    : rng{rng}
    , walls{walls}
    , root{&parent->addChild<Object3D>()}
    {
    }

    virtual void init() = 0;

    virtual void generate() = 0;

    virtual void rotateCCW(int previousPlatformWidth)
    {
        root->rotateYLocal(90.0_degf);
        root->translateLocal({-1, 0, -1});
    }

    virtual void rotateCW(int previousPlatformWidth)
    {
        root->rotateYLocal(-90.0_degf);
        root->translateLocal({float(previousPlatformWidth) - 1, 0, -float(width) + 1});
    }

    virtual Object3D * anchorPoint() { return nextPlatformAnchor; }

    virtual void addFloor()
    {
        MagnumAABB floor{*root, {0, 0, 0, length, 1, width}};
        layoutBoxes.emplace_back(floor);

        nextPlatformAnchor = &root->addChild<Object3D>();
        nextPlatformAnchor->translateLocal({float(length), 0, 0});

        boundingBoxDirty = true;
    }

    virtual void addWalls()
    {
        if (walls & WALLS_SOUTH)
            wallBoxes.emplace_back(MagnumAABB{*root, {0, 0, 0, 1, height, width}});
        if (walls & WALLS_NORTH)
            wallBoxes.emplace_back(MagnumAABB{*root, {length - 1, 0, 0, length, height, width}});
        if (walls & WALLS_EAST)
            wallBoxes.emplace_back(MagnumAABB{*root, {0, 0, 0, length, height, 1}});
        if (walls & WALLS_WEST)
            wallBoxes.emplace_back(MagnumAABB{*root, {0, 0, width - 1, length, height, width}});

        boundingBoxDirty = true;
    }

    virtual BoundingBox platformBoundingBox()
    {
        if (!boundingBoxDirty)
            return outerBoundingBox;

        if (!layoutBoxes.empty())
            outerBoundingBox = layoutBoxes.front().boundingBox();
        else if (!wallBoxes.empty())
            outerBoundingBox = wallBoxes.front().boundingBox();
        else
            outerBoundingBox = BoundingBox{};

        boundingBoxDirty = false;

        for (auto &boxes : {layoutBoxes, wallBoxes})
            for (auto &box : boxes) {
                const auto bb = box.boundingBox();
                outerBoundingBox.addPoint(bb.min);
                outerBoundingBox.addPoint(bb.max);
            }

        return outerBoundingBox;
    }

    virtual bool collidesWith(Platform &other)
    {
        return platformBoundingBox().collidesWith(other.platformBoundingBox());
    }

public:
    Rng &rng;

    int walls{};

    // length = x, height = y, width = z
    int length{}, height{}, width{};

    std::vector<MagnumAABB> layoutBoxes, wallBoxes;
    std::map<TerrainType, std::vector<MagnumAABB>> terrainBoxes;

    Object3D *nextPlatformAnchor{}, *root{};

    bool boundingBoxDirty = true;
    BoundingBox outerBoundingBox;
};

class EmptyPlatform : public Platform
{
public:
    explicit EmptyPlatform(Object3D *parent, Rng &rng, int walls, int w = -1)
    : Platform{parent, rng, walls}
    {
        width = w;
    }

    void init() override
    {
        length = randRange(4, 14, rng);
        if (width == -1)
            width = randRange(4, 12, rng);

        // height = randRange(3, 7, rng);
        height = 4;
    }

    void generate() override
    {
        addFloor();
        addWalls();
    }
};

class WallPlatform : public EmptyPlatform
{
public:
    explicit WallPlatform(Object3D *parent, Rng &rng, int walls, int w = -1)
    : EmptyPlatform{parent, rng, walls, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();

        wallHeight = randRange(1, std::min(3, height - 1), rng);
        height = randRange(wallHeight + 2, wallHeight + 5, rng);
    }

    void generate() override
    {
        EmptyPlatform::generate();

        const auto wallX = randRange(1, length, rng);
        const auto wallThickness = randRange(1, length - wallX + 1, rng);

        MagnumAABB wall{*root, {wallX, 1, 1, wallX + wallThickness, 1 + wallHeight, width - 1}};
        layoutBoxes.emplace_back(wall);
    }

private:
    int wallHeight{};
};

class LavaPlatform : public EmptyPlatform
{
public:
    explicit LavaPlatform(Object3D *parent, Rng &rng, int walls, int w = -1)
        : EmptyPlatform{parent, rng, walls, w}
    {
    }

    void generate() override
    {
        EmptyPlatform::generate();

        const auto lavaLength = randRange(2, std::min(4, length - 1), rng);
        const auto lavaX = randRange(1, length - lavaLength, rng);

        MagnumAABB lava{*root, {lavaX, 1, 1, lavaX + lavaLength, 2, width - 1}};
        terrainBoxes[TERRAIN_LAVA].emplace_back(lava);
    }
};

class StepPlatform : public EmptyPlatform
{
public:
    explicit StepPlatform(Object3D *parent, Rng &rng, int walls, int w = -1)
        : EmptyPlatform{parent, rng, walls, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();

        stepHeight = randRange(1, 4, rng);
        height = randRange(stepHeight + 3, stepHeight + 5, rng);
    }

    void generate() override
    {
        const auto stepX = randRange(1, length, rng);
        MagnumAABB floorLow{*root, {0, 0, 0, stepX + 1, 1, width}};
        MagnumAABB floorHigh{*root, {stepX, stepHeight, 0, length, stepHeight + 1, width}};
        MagnumAABB wall{*root, {stepX, 0, 0, stepX + 1, stepHeight + 1, width}};

        layoutBoxes.emplace_back(floorLow);
        layoutBoxes.emplace_back(floorHigh);
        layoutBoxes.emplace_back(wall);

        nextPlatformAnchor = &root->addChild<Object3D>();
        nextPlatformAnchor->translateLocal({float(length), float(stepHeight), 0});

        addWalls();
    }

private:
    int stepHeight{};
};


class GapPlatform : public EmptyPlatform
{
public:
    explicit GapPlatform(Object3D *parent, Rng &rng, int walls, int w = -1)
        : EmptyPlatform{parent, rng, walls, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();

        gap = randRange(2, std::min(4, length - 1), rng);
        gapX = randRange(1, length - gap, rng);
    }

    void generate() override
    {
        MagnumAABB floorStart{*root, {0, 0, 0, gapX, 1, width}};
        MagnumAABB floorEnd{*root, {gapX + gap, 0, 0, length, 1, width}};

        layoutBoxes.emplace_back(floorStart);
        layoutBoxes.emplace_back(floorEnd);

        nextPlatformAnchor = &root->addChild<Object3D>();
        nextPlatformAnchor->translateLocal({float(length), 0, 0});

        addWalls();
    }

private:
    int gap{}, gapX{};
};

class StartPlatform : public EmptyPlatform
{
public:
    explicit StartPlatform(Object3D *parent, Rng &rng, int w = -1)
    : EmptyPlatform{parent, rng, WALLS_SOUTH | WALLS_EAST | WALLS_WEST, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();

        assert(length < maxLength);
        assert(width < maxWidth);
    }

    std::vector<VoxelCoords> generateMovableBoxes(int numObjects)
    {
        std::vector<VoxelCoords> boxes;

        for (int i = 0; i < numObjects; ++i) {
            const int x = randRange(1, length, rng);
            const int z = randRange(1, width - 1, rng);
            const int y = ++occupancy[x][z];
            boxes.emplace_back(x, y, z);
        }

        return boxes;
    }

    std::vector<VoxelCoords> agentSpawnPoints(int numAgents)
    {
        std::vector<VoxelCoords> spawnPoints;
        std::set<std::pair<int, int>> used;

        for (int i = 0; i < numAgents; ++i) {
            int x, y, z;

            for (int attempt = 0; attempt < 10; ++attempt) {
                x = randRange(1, length, rng);
                z = randRange(1, width - 1, rng);
                if (used.count({x, z}))
                    continue;

                y = ++occupancy[x][z];
                spawnPoints.emplace_back(x, y, z);
                used.emplace(x, z);
                break;
            }
        }

        return spawnPoints;
    }

private:
    constexpr static int maxLength = 21, maxWidth = 21;

    int occupancy[maxLength][maxWidth] = {{0}};
};

class ExitPlatform : public EmptyPlatform
{
public:
    explicit ExitPlatform(Object3D *parent, Rng &rng, int w = -1)
    : EmptyPlatform{parent, rng, WALLS_NORTH | WALLS_EAST | WALLS_WEST, w}
    {
    }

    void generate() override
    {
        EmptyPlatform::generate();

        MagnumAABB exit{*root, {length - 3, 1, 1, length - 1, 2, width - 1}};
        terrainBoxes[TERRAIN_EXIT].emplace_back(exit);
    }
};

class TransitionPlatform : public EmptyPlatform
{
public:
    explicit TransitionPlatform(Object3D *parent, Rng &rng, int walls, int l, int w)
    : EmptyPlatform{parent, rng, walls, -1}
    {
        length = l, width = w;
    }

    void init() override { height = 4; }
};


std::unique_ptr<Platform> makePlatform(Object3D *parent, Rng &rng, int walls, int width)
{
    enum { EMPTY, WALL, LAVA, STEP, GAP };
    const static std::vector<int> supportedPlatforms = {EMPTY, WALL, LAVA, STEP, GAP};
    const auto platformType = randomSample(supportedPlatforms, rng);

    switch (platformType) {
        case STEP:
            return std::make_unique<StepPlatform>(parent, rng, walls, width);
        case GAP:
            return std::make_unique<GapPlatform>(parent, rng, walls, width);
        case LAVA:
            return std::make_unique<LavaPlatform>(parent, rng, walls, width);
        case WALL:
            return std::make_unique<WallPlatform>(parent, rng, walls, width);
        case EMPTY:
        default:
            return std::make_unique<EmptyPlatform>(parent, rng, walls, width);
    }
}

}

ObstaclesScenario::ObstaclesScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, vg{*this}
, gridLayout{*this, envState.rng}
{
}

void ObstaclesScenario::reset()
{
    vg.reset(env, envState);
    agentSpawnPositions.clear(), objectSpawnPositions.clear(), terrain.clear();

    const bool drawWalls = randRange(0, 2, envState.rng);
    std::vector<std::unique_ptr<Platform>> platforms;

    StartPlatform *startPlatform = nullptr;

    for (int attempt = 0; attempt < 20; ++attempt) {
        platforms.clear();

        int numPlatforms = randRange(2, 10, envState.rng);

        enum { STRAIGHT, TURN_LEFT, TURN_RIGHT };

        static const std::vector<int> orientations = {STRAIGHT, TURN_LEFT, TURN_RIGHT};

        levelRoot = std::make_unique<Object3D>(nullptr);

        auto startPlatformPtr = std::make_unique<StartPlatform>(levelRoot.get(), envState.rng);
        startPlatformPtr->init(), startPlatformPtr->generate();
        startPlatform = startPlatformPtr.get();
        platforms.emplace_back(std::move(startPlatformPtr));
        int requiredWidth = startPlatform->width;

        Platform *previousPlatform = startPlatform;

        for (int i = 0; i < numPlatforms; ++i) {
            auto orientation = randomSample(orientations, envState.rng);
            requiredWidth = orientation == STRAIGHT ? requiredWidth : -1;

            platforms.emplace_back(makePlatform(previousPlatform->nextPlatformAnchor, envState.rng, WALLS_WEST | WALLS_EAST, requiredWidth));
            auto platform = platforms.back().get();

            platform->init(), platform->generate();

            switch (orientation) {
                case STRAIGHT:
                    break;
                case TURN_LEFT:
                    platform->rotateCCW(previousPlatform->width);
                    break;
                case TURN_RIGHT:
                    platform->rotateCW(previousPlatform->width);
                    break;
                default:
                    break;
            }

            if (orientation != STRAIGHT) {
                int walls = WALLS_NORTH;
                walls |= orientation == TURN_LEFT ? WALLS_WEST : WALLS_EAST;
                const int w = previousPlatform->width, l = platform->width - 1;

                platforms.emplace_back(std::make_unique<TransitionPlatform>(previousPlatform->nextPlatformAnchor, envState.rng, walls, l, w));
                auto transitionPlatform = platforms.back().get();

                transitionPlatform->init();
                transitionPlatform->generate();
            }

            previousPlatform = platform;
            requiredWidth = platform->width;
        }

        auto exitPlatformPtr = std::make_unique<ExitPlatform>(previousPlatform->nextPlatformAnchor, envState.rng, requiredWidth);
        exitPlatformPtr->init(), exitPlatformPtr->generate();
        platforms.emplace_back(std::move(exitPlatformPtr));

        // don't check collisions with self and with the previous platforms (there might be overlap)
        bool selfCollision = false;
        for (int j = 0; j < int(platforms.size()) && !selfCollision; ++j) {
            for (int k = 0; k < j - 2; ++k) {
                if (platforms[j]->collidesWith(*platforms[k])) {
                    TLOG(INFO) << "Platform " << j << " collides with " << k;
                    selfCollision = true;
                    break;
                }
            }
        }

        if (selfCollision)
            TLOG(INFO) << "Self collision! Attempt " << attempt << " regenerate!";
        else {
            break;
        }
    }


    for (auto &p : platforms) {
        for (auto &bb : p->layoutBoxes)
            vg.addBoundingBox(bb.boundingBox(), VOXEL_OPAQUE | VOXEL_SOLID);
        for (auto &bb : p->wallBoxes)
            vg.addBoundingBox(bb.boundingBox(), VOXEL_SOLID | (drawWalls ? VOXEL_OPAQUE : 0));

        for (auto &[k, v] : p->terrainBoxes)
            for (auto &bb : v) {
                terrain[k].emplace_back(bb.boundingBox());
                vg.addTerrainBoundingBox(bb.boundingBox(), TERRAIN_EXIT);
            }
    }

    agentSpawnPositions = startPlatform->agentSpawnPoints(env.getNumAgents());
}

void ObstaclesScenario::step()
{
    int numAgentsAtExit = 0;
    for (int i = 0; i < env.getNumAgents(); ++i) {
        auto agent = envState.agents[i];
        const auto &t = agent->absoluteTransformation().translation();
        const auto voxel = vg.grid.getCoords(t);
        if (vg.grid.hasVoxel(voxel)) {
            const auto terrainType = vg.grid.get(voxel)->terrain;
            if (terrainType & TERRAIN_EXIT)
                ++numAgentsAtExit;
        }
    }

    if (numAgentsAtExit == env.getNumAgents()) {
        envState.done = true;
        // TODO reward
    }

//    TLOG(INFO) << numAgentsAtExit;
}

void ObstaclesScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();

    // TODO this is ugly, refactor
    for (auto &[voxelType, bb] : boundingBoxesByType)
        gridLayout.addBoundingBoxes(drawables, envState, bb, voxelType);

    // add terrains
    for (auto &[k, v] : terrain)
        for (auto &bb : v)
            addTerrain(drawables, k, bb);
}

// TODO move to reusable component
void ObstaclesScenario::addTerrain(DrawablesMap &drawables, TerrainType type, const BoundingBox &bb)
{
    const auto scale = Vector3(bb.max.x() - bb.min.x(), 1.0, bb.max.z() - bb.min.z());

    if (scale.x() > 0) {
        // otherwise we don't draw anything
        const auto pos = Magnum::Vector3(bb.min.x() + scale.x() / 2, bb.min.y(), bb.min.z() + scale.z() / 2);

        auto &terrainObject = envState.scene->addChild<Object3D>(envState.scene.get());
        terrainObject.scale({0.5, 0.025, 0.5}).scale(scale);
        terrainObject.translate({0.0, 0.025, 0.0});
        terrainObject.translate(pos);

        drawables[DrawableType::Box].emplace_back(&terrainObject, rgb(terrainColor(type)));
    }
}

// TODO
ColorRgb ObstaclesScenario::terrainColor(TerrainType type)
{
    const static std::map<TerrainType, ColorRgb> colors = {
        {TerrainType::TERRAIN_EXIT, ColorRgb::EXIT_PAD},
        {TerrainType::TERRAIN_LAVA, ColorRgb::RED},
    };

    return colors.at(type);
}
