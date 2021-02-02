#pragma once

#include <set>
#include <map>
#include <cassert>

#include <util/util.hpp>
#include <util/magnum.hpp>
#include <util/voxel_grid.hpp>
#include <util/math_utils.hpp>
#include <util/tiny_logger.hpp>

#include <env/env.hpp>

#include <scenarios/const.hpp>


namespace VoxelWorld
{

enum PlatformOrientation
{
    ORIENTATION_STRAIGHT,
    ORIENTATION_TURN_LEFT,
    ORIENTATION_TURN_RIGHT,
};

enum TerrainType
{
    TERRAIN_EXIT = 1,
    TERRAIN_LAVA = 1 << 1,
    TERRAIN_BUILDING_ZONE = 1 << 2,
};

enum {
    WALLS_SOUTH = 1,
    WALLS_NORTH = 1 << 1,
    WALLS_WEST = 1 << 2,
    WALLS_EAST = 1 << 3,

    WALLS_NONE = 0,
    WALLS_ALL = WALLS_SOUTH | WALLS_NORTH | WALLS_EAST | WALLS_WEST,
};


inline ColorRgb terrainColor(TerrainType type)
{
    const static std::map<TerrainType, ColorRgb> colors = {
        {TerrainType::TERRAIN_EXIT, ColorRgb::EXIT_PAD},
        {TerrainType::TERRAIN_LAVA, ColorRgb::RED},
        {TerrainType::TERRAIN_BUILDING_ZONE, ColorRgb::BUILDING_ZONE},
    };

    return colors.at(type);
}


struct BoundingBox
{
    BoundingBox() = default;

    BoundingBox(const VoxelCoords &min, const VoxelCoords &max)
        : min{min}, max{max}
    {
    }

    BoundingBox(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        : min{minX, minY, minZ}
        , max{maxX, maxY, maxZ}
    {
    }

    void addPoint(const VoxelCoords &v)
    {
        if (v.x() < min.x()) min.x() = v.x();
        if (v.x() > max.x()) max.x() = v.x();

        if (v.y() < min.y()) min.y() = v.y();
        if (v.y() > max.y()) max.y() = v.y();

        if (v.z() < min.z()) min.z() = v.z();
        if (v.z() > max.z()) max.z() = v.z();
    }

    void sort()
    {
        // sort the min/max vertices after the transformation
        // this only works for rotations in 90-degree increments
        if (min.x() > max.x()) std::swap(min.x(), max.x());
        if (min.y() > max.y()) std::swap(min.y(), max.y());
        if (min.z() > max.z()) std::swap(min.z(), max.z());
    }

    [[nodiscard]] bool collidesWith(const BoundingBox &other) const
    {
        // we're looking for an axis with no overlap
        if (max.x() <= other.min.x()) return false;
        if (min.x() >= other.max.x()) return false;
        if (max.y() <= other.min.y()) return false;
        if (min.y() >= other.max.y()) return false;
        if (max.z() <= other.min.z()) return false;
        if (min.z() >= other.max.z()) return false;

        return true;
    }

public:
    VoxelCoords min, max;
};


struct MagnumAABB
{
    MagnumAABB(Object3D &parent, const BoundingBox &bb)
    {
        min = &parent.addChild<Object3D>();
        min->translateLocal(Magnum::Vector3(bb.min));

        max = &parent.addChild<Object3D>();
        max->translateLocal(Magnum::Vector3(bb.max));
    }

    [[nodiscard]] BoundingBox boundingBox() const
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
    explicit Platform(Object3D *parent, Rng &rng, int walls, const FloatParams &params)
    : rng{rng}
    , walls{walls}
    , root{&parent->addChild<Object3D>()}
    , params{params}
    {
    }
    virtual ~Platform() = default;

    virtual void init() = 0;

    virtual void generate() = 0;

    virtual void rotateCCW(int /*previousPlatformWidth*/)
    {
        root->rotateYLocal(degrees(90.0));
        root->translateLocal({-1, 0, -1});
    }

    virtual void rotateCW(int previousPlatformWidth)
    {
        root->rotateYLocal(degrees(-90.0));
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

    virtual std::vector<Magnum::Vector3> agentSpawnPoints(int numAgents)
    {
        std::vector<Magnum::Vector3> spawnPoints;
        std::set<std::pair<int, int>> used;

        for (int i = 0; i < numAgents; ++i) {
            int x, y, z;

            for (int attempt = 0; attempt < 10; ++attempt) {
                x = randRange(1, length - 1, rng);
                z = randRange(1, width - 1, rng);
                if (used.count({x, z}))
                    continue;

                y = occupancy[{x, z}] + 1;
                occupancy[{x, z}] += 2;
                spawnPoints.emplace_back(x, y, z);
                used.emplace(x, z);
                break;
            }
        }

        return spawnPoints;
    }

    virtual int requiresMovableBoxesToTraverse() { return 0; }

    virtual std::vector<VoxelCoords> generateMovableBoxes(int numBoxesToGenerate)
    {
        std::vector<VoxelCoords> boxes;
        constexpr int maxAttempts = 10;

        for (int i = 0; i < numBoxesToGenerate; ++i) {
            for (int attempt = 0; attempt < 10; ++attempt) {
                const int x = randRange(1, length - 1, rng);
                const int z = randRange(1, width - 1, rng);
                if (occupancy[{x, z}] < 2 || attempt >= maxAttempts - 1) {
                    const int y = ++occupancy[{x, z}];
                    boxes.emplace_back(x, y, z);
                    break;
                }
            }
        }

        return adjustTransformation(boxes);
    }

    std::vector<VoxelCoords> adjustTransformation(std::vector<VoxelCoords> &coords) const
    {
        for (auto &c : coords) {
            Magnum::Vector3 v{c.x() + 0.5f, c.y() + 0.5f, c.z() + 0.5f};
            c = toVoxel(root->absoluteTransformation().transformPoint(v));
        }

        return coords;
    }

    int param(const std::string &str) const
    {
        return int(lround(params.at(str)));
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

    std::map<std::pair<int, int>, int> occupancy;

    const FloatParams &params;
};

class EmptyPlatform : public Platform
{
public:
    explicit EmptyPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int w = -1)
        : Platform{parent, rng, walls, params}
    {
        width = w;
    }

    void init() override
    {
        length = randRange(4, 10, rng);
        if (width == -1)
            width = randRange(5, 9, rng);

        // height = randRange(3, 7, rng);
        height = 5;
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
    explicit WallPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int w = -1)
        : EmptyPlatform{parent, rng, walls, params, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();

        wallHeight = randRange(1, 5, rng);
        height = randRange(wallHeight + 4, wallHeight + 6, rng);
    }

    void generate() override
    {
        EmptyPlatform::generate();

        const auto wallX = randRange(1, length, rng);
        const auto wallThickness = randRange(1, length - wallX + 1, rng);

        MagnumAABB wall{*root, {wallX, 1, 1, wallX + wallThickness, 1 + wallHeight, width - 1}};
        layoutBoxes.emplace_back(wall);

        // this way we won't generate boxes on top of the wall (for the most part)
        for (int x = wallX; x < wallX + wallThickness; ++x)
            for (int z = 1; z < width; ++z)
                occupancy[{x, z}] = wallHeight;
    }

    int requiresMovableBoxesToTraverse() override { return triangularNumber(wallHeight - 1); }

private:
    int wallHeight{};
};

class LavaPlatform : public EmptyPlatform
{
public:
    explicit LavaPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int w = -1)
        : EmptyPlatform{parent, rng, walls, params, w}
    {
    }

    void generate() override
    {
        EmptyPlatform::generate();

        lavaLength = randRange(2, std::min(4, length - 1), rng);
        const auto lavaX = randRange(1, length - lavaLength, rng);

        MagnumAABB lava{*root, {lavaX, 1, 1, lavaX + lavaLength, 2, width - 1}};
        terrainBoxes[TERRAIN_LAVA].emplace_back(lava);
    }

    int requiresMovableBoxesToTraverse() override { return std::max(0, std::min(2, lavaLength - 2)); }

private:
    int lavaLength{};
};

class StepPlatform : public EmptyPlatform
{
public:
    explicit StepPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int w = -1)
        : EmptyPlatform{parent, rng, walls, params, w}
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

        // this way we won't generate boxes on top of the step (for the most part)
        for (int x = stepX + 1; x < length; ++x)
            for (int z = 1; z < width; ++z)
                occupancy[{x, z}] = stepHeight;
    }

    int requiresMovableBoxesToTraverse() override { return triangularNumber(stepHeight - 1); }

private:
    int stepHeight{};
};


class GapPlatform : public EmptyPlatform
{
public:
    explicit GapPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int w = -1)
        : EmptyPlatform{parent, rng, walls, params, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();

        gap = randRange(param(Str::obstaclesMinGap), std::min(param(Str::obstaclesMaxGap) + 1, length - 1), rng);
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

    int requiresMovableBoxesToTraverse() override { return triangularNumber(std::max(0, gap - 2)); }

    std::vector<VoxelCoords> generateMovableBoxes(int numBoxesToGenerate) override
    {
        std::vector<VoxelCoords> boxes, candidates;

        for (int x = 0; x < length; ++x)
            for (int z = 1; z < width - 1; ++z) {
                if (x >= gapX && x < gapX + gap)
                    continue;
                candidates.emplace_back(x, 1, z);
            }

        for (int i = 0; i < numBoxesToGenerate; ++i) {
            const auto v = randomSample(candidates, rng);
            const int y = ++occupancy[{v.x(), v.z()}];
            boxes.emplace_back(v.x(), y, v.z());
        }

        return adjustTransformation(boxes);
    }

private:
    int gap{}, gapX{};
};

class StartPlatform : public EmptyPlatform
{
public:
    explicit StartPlatform(Object3D *parent, Rng &rng, const FloatParams &params, int w = -1)
        : EmptyPlatform{parent, rng, WALLS_SOUTH | WALLS_EAST | WALLS_WEST, params, w}
    {
    }

    void init() override
    {
        EmptyPlatform::init();
    }
};

class ExitPlatform : public EmptyPlatform
{
public:
    explicit ExitPlatform(Object3D *parent, Rng &rng, const FloatParams &params, int w = -1)
        : EmptyPlatform{parent, rng, WALLS_NORTH | WALLS_EAST | WALLS_WEST, params, w}
    {
    }

    void generate() override
    {
        EmptyPlatform::generate();

        MagnumAABB exit{*root, {length - 3, 1, 1, length - 1, 3, width - 1}};
        terrainBoxes[TERRAIN_EXIT].emplace_back(exit);
    }
};

class TransitionPlatform : public EmptyPlatform
{
public:
    explicit TransitionPlatform(Object3D *parent, Rng &rng, int walls, const FloatParams &params, int l, int w)
        : EmptyPlatform{parent, rng, walls, params, -1}
    {
        length = l, width = w;
    }

    void init() override { height = 5; }
};

}
