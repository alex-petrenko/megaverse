#include <set>
#include <queue>

#include <scenarios/scenario_rearrange.hpp>


using namespace Magnum;
using namespace VoxelWorld;


class RearrangeScenario::RearrangePlatform : public EmptyPlatform
{
public:
    explicit RearrangePlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int)
    : EmptyPlatform(parent, rng, walls, params)
    {
    }

    ~RearrangePlatform() override = default;

    void init() override
    {
        height = 8;
        length = 19;
        width = 14;
    }

    void generate() override
    {
        EmptyPlatform::generate();
    }
};


RearrangeScenario::RearrangeScenario(const std::string &name, Env &env, Env::EnvState &envState)
: DefaultScenario(name, env, envState)
, platformsComponent{*this}
, vg{*this, 100, 0, 0, 0, 1}
, objectStackingComponent{*this, env.getNumAgents(), vg.grid, *this}
{
    std::map<std::string, float> rewardShapingScheme{
        {Str::rearrangeOneObjectCorrectPosition, 1.0f},
        {Str::rearrangeAllObjectsCorrectPosition, 5.0f},
    };

    for (int i = 0; i < env.getNumAgents(); ++i)
        rewardShaping[i] = rewardShapingScheme;
}

RearrangeScenario::~RearrangeScenario() = default;


void RearrangeScenario::reset()
{
    solved = false;

    vg.reset(env, envState);
    objectStackingComponent.reset(env, envState);
    platformsComponent.reset(env, envState);

    platform = std::make_unique<RearrangePlatform>(platformsComponent.levelRoot.get(), envState.rng, WALLS_ALL, floatParams, env.getNumAgents());
    platform->init(), platform->generate();
    vg.addPlatform(*platform, false);

    arrangement = Arrangement{};
    arrangementObjects.clear();

    generateArrangement();
}

void RearrangeScenario::generateArrangement()
{
    const int arrangementSize = randRange(2, 8, envState.rng);

    // bfs
    std::queue<ArrangementItem> q;
    std::unordered_set<VoxelCoords> used;

    const auto firstItem = ArrangementItem::random(envState.rng, {0, 0, 0});
    q.push(firstItem);
    arrangement.items.emplace_back(firstItem);
    used.insert({0, 0, 0});

    std::vector<VoxelCoords> directions{
        {-1, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, -1},
        {0, 0, 1},
    };

    while (!q.empty()) {
        const auto currItem = q.front();
        q.pop();

        int maxBranches = randRange(1, int(directions.size()) + 1, envState.rng);
        maxBranches = randRange(1, maxBranches + 1, envState.rng);
        int numBranches = 0;

        std::shuffle(directions.begin(), directions.end(), envState.rng);

        for (auto dir : directions) {
            const auto newOffset = currItem.offset + dir;
            const auto below = newOffset - VoxelCoords {0, 1, 0};

            if (newOffset.y() >= 2 || abs(newOffset.x()) >= 2 || abs(newOffset.z()) >= 2)
                continue;

            if (used.count(newOffset))
                continue;

            // item has to be on the floor or on top of another item
            if (!(newOffset.y() == 0 || used.count(below)))
                continue;

            const auto newItem = ArrangementItem::random(envState.rng, newOffset);
            q.push(newItem);
            arrangement.items.emplace_back(newItem);
            used.insert(newOffset);
            ++numBranches;
            if (numBranches >= maxBranches)
                break;

            if (int(arrangement.items.size()) >= arrangementSize)
                break;
        }

        if (int(arrangement.items.size()) >= arrangementSize)
            break;
    }
}

void RearrangeScenario::step()
{
    objectStackingComponent.step(env, envState);
}

bool RearrangeScenario::canPlaceObject(int, const VoxelCoords &coord, Object3D *)
{
    const auto delta = coord - rightCenter;
    return abs(delta.x()) <= 2 && abs(delta.z()) <= 2;
}

int RearrangeScenario::countMatchingObjects() const
{
    int matching = 0;
    for (const auto *obj : arrangementObjects) {
        if (obj->pickedUp)
            continue;

        const auto voxelCoords = vg.grid.getCoords(obj->absoluteTransformation().translation());
        const auto offset = voxelCoords - rightCenter;
        if (arrangement.contains(obj->arrangementItem.shape, obj->arrangementItem.color, offset))
            ++matching;
    }

    TLOG(DEBUG) << "Matching " << matching << " objects";

    return matching;
}

void RearrangeScenario::placedObject(int agentIdx, const VoxelCoords &, Object3D *obj)
{
    dynamic_cast<ArrangementObject *>(obj)->pickedUp = false;
    checkDone(agentIdx);
}

void RearrangeScenario::pickedObject(int agentIdx, const VoxelCoords &, Object3D *obj)
{
    dynamic_cast<ArrangementObject *>(obj)->pickedUp = true;
    checkDone(agentIdx);
}

void RearrangeScenario::checkDone(int agentIdx)
{
    const auto matches = countMatchingObjects();
    const auto delta = matches - matchingObjects;
    envState.lastReward[agentIdx] += delta * rewardShaping[agentIdx].at(Str::rearrangeOneObjectCorrectPosition);

    matchingObjects = matches;

    if (matches >= int(arrangement.items.size()) && !solved) {
        solved = true;
        envState.lastReward[agentIdx] += rewardShaping[agentIdx].at(Str::rearrangeAllObjectsCorrectPosition);
        envState.currEpisodeSec = episodeLengthSec() - 0.5f;  // terminate in 0.5 second
    }
}

std::vector<Magnum::Vector3> RearrangeScenario::agentStartingPositions()
{
    auto positions = std::vector<Magnum::Vector3>(env.getNumAgents());

    for (int i = 0; i < env.getNumAgents(); ++i)
        positions[i] = Vector3 {float(platform->length) / 2 + float(i), 1, float(platform->width) - 3};

    return positions;
}

void RearrangeScenario::arrangementDrawables(DrawablesMap &drawables, const Arrangement &arr, VoxelCoords center, bool interactive)
{
    const static std::map<DrawableType, Vector3> scales {
        { DrawableType::Sphere, {1, 1, 1}},
        { DrawableType::Box, {1, 1, 1}},
        { DrawableType::Capsule, {0.8, 0.5, 0.8}},
        { DrawableType::Cylinder, {0.9, 2, 0.9}},
    };

    const auto objSize = 0.45f;

    std::unordered_set<VoxelCoords> occupied;
    for (const auto &item : arr.items)
        occupied.insert(item.offset);

    int numUnmovedItems = arr.items.size();
    if (interactive)
        numUnmovedItems = randRange(0, arr.items.size(), envState.rng);

    int placedItems = 0;

    for (const auto &item : arr.items) {
        auto pos = item.offset + center;

        if (interactive && placedItems >= numUnmovedItems) {
            // random offset
            VoxelCoords newOffset = item.offset;
            while (occupied.count(newOffset))
                newOffset = VoxelCoords{randRange(-2, 3, envState.rng), 0, randRange(-2, 3, envState.rng)};

            pos = newOffset + center;
            occupied.insert(newOffset);
        }

        auto translation = Magnum::Vector3{float(pos.x()) + 0.5f, float(pos.y()) + 0.5f, float(pos.z()) + 0.5f};

        auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});

        auto &object = envState.scene->addChild<ArrangementObject>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
        object.arrangementItem = item;

        object.scale(scales.at(item.shape) * objSize).translate(translation);
        if (item.shape == DrawableType::Cylinder)
            object.setCollisionScale({1, 0.5, 1});
        else if (item.shape == DrawableType::Capsule)
            object.setCollisionScale({1, 2, 1});

        object.syncPose();

        drawables[item.shape].emplace_back(&object, rgb(item.color));

        envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));

        if (interactive) {
            VoxelRearrange voxelState;
            voxelState.physicsObject = &object;
            vg.grid.set(pos, voxelState);

            arrangementObjects.emplace_back(&object);
        }

        ++placedItems;
    }
}

void RearrangeScenario::addEpisodeDrawables(DrawablesMap &drawables)
{
    auto boundingBoxesByType = vg.toBoundingBoxes();
    for (auto &[voxelType, bb] : boundingBoxesByType)
        addBoundingBoxes(drawables, envState, bb, voxelType, 1);

    for (int dx = -3; dx <= 3; ++dx)
        for (int dz = -3; dz <= 3; ++dz) {
            VoxelRearrange voxelState;
            voxelState.voxelType = VOXEL_SOLID;
            vg.grid.set(VoxelCoords {leftCenter.x() + dx, 1, leftCenter.z() + dz}, voxelState);
            vg.grid.set(VoxelCoords {rightCenter.x() + dx, 1, rightCenter.z() + dz}, voxelState);
        }

    arrangementDrawables(drawables, arrangement, leftCenter, false);
    arrangementDrawables(drawables, arrangement, rightCenter, true);

    matchingObjects = countMatchingObjects();

    Vector3 platformCenter = {9.5, 0, 7};

    addStaticCollidingBox(drawables, envState, {8.35, 0.5, 5.65}, Vector3{platformCenter} + Vector3 {0.0, 1, 0.0}, ColorRgb::DARK_GREY);

    // add pedestal for the desired arrangement
    addStaticCollidingBox(drawables, envState, {3, 0.5, 3}, Vector3{leftCenter} + Vector3 {0.5, -0.5, 0.5}, ColorRgb::LAYOUT);
    addStaticCollidingBox(drawables, envState, {1.5, 0.5, 1.5}, Vector3{leftCenter} + Vector3 {0.5, -0.49, 0.5}, ColorRgb::DARK_GREY);
    addStaticCollidingBox(drawables, envState, {3, 0.5, 3}, Vector3{leftCenter} + Vector3 {1.0, -0.66, 1.0}, ColorRgb::LAYOUT);
    addStaticCollidingBox(drawables, envState, {3, 0.5, 3}, Vector3{leftCenter} + Vector3 {1.5, -0.82, 1.5}, ColorRgb::LAYOUT);

    // add pedestal for the working area
    addStaticCollidingBox(drawables, envState, {3, 0.5, 3}, Vector3{rightCenter} + Vector3 {0.5, -0.5, 0.5}, ColorRgb::BLUE);
    addStaticCollidingBox(drawables, envState, {1.5, 0.5, 1.5}, Vector3{rightCenter} + Vector3 {0.5, -0.49, 0.5}, ColorRgb::DARK_GREY);
    addStaticCollidingBox(drawables, envState, {3, 0.5, 3}, Vector3{rightCenter} + Vector3 {0, -0.66, 1.0}, ColorRgb::BLUE);
    addStaticCollidingBox(drawables, envState, {3, 0.5, 3}, Vector3{rightCenter} + Vector3 {-0.5, -0.82, 1.5}, ColorRgb::BLUE);
}
