#include <queue>

#include <cassert>
#include <unordered_set>

#include <util/tiny_logger.hpp>

#include <env/layout_generator.hpp>

#include <util/util.hpp>
#include <util/magnum.hpp>


// Utils

namespace
{

Magnum::Vector3i directions[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};


struct CoordRange
{
    int min, max;
};


CoordRange startEndCoord(int bboxMin, int bboxMax, int direction)
{
    if (direction == 1)
        return {bboxMax + 1, bboxMax + 1};
    else if (direction == -1)
        return {bboxMin - 1, bboxMin - 1};
    else
        return {bboxMin, bboxMax};
}

std::vector<VoxelCoords> getFreeVoxels(const VoxelGrid<VoxelState> &grid, int length, int width, int startY)
{
    std::vector<VoxelCoords> res;

    for (int x = 1; x < length - 1; ++x)
        for (int z = 1; z < width - 1; ++z) {
            for (int y = startY; y > 0; --y) {
                const VoxelCoords coords{x, y - 1, z};
                const auto v = grid.get(coords);
                if (v && v->solid) {
                    res.emplace_back(x, y, z);
                    break;
                }
            }
        }

    return res;
}

}

// Layout generators

class LayoutGenerator::LayoutGeneratorImpl
{
public:
    explicit LayoutGeneratorImpl(Rng &rng)
    : rng{rng}
    {
    }

    virtual void init()
    {
        length = randRange(8, 30, rng);
        width = randRange(5, 25, rng);
    }

    virtual void generate(VoxelGrid<VoxelState> &grid) = 0;

    virtual std::vector<BoundingBox> extractPrimitives(VoxelGrid<VoxelState> &grid) = 0;

    virtual BoundingBox levelExit(const VoxelGrid<VoxelState> &grid, int numAgents) = 0;

    virtual std::vector<VoxelCoords> startingPositions(const VoxelGrid<VoxelState> &grid, int numAgents) = 0;

public:
    Rng &rng;

    // length = x, height = y, width = z
    int length{}, height{}, width{};
};


class LayoutGeneratorBasic : public LayoutGenerator::LayoutGeneratorImpl
{
public:
    explicit LayoutGeneratorBasic(Rng &rng)
    : LayoutGeneratorImpl(rng)
    {

    }

    void init() override
    {
        LayoutGeneratorImpl::init();

        height = randRange(3, 5, rng);
    }

    void generate(VoxelGrid<VoxelState> &grid) override
    {
        // floor

        for (int x = 0; x < length; ++x)
            for (int z = 0; z < width; ++z)
                grid.set({x, 0, z}, {true});

        // generating the perimeter walls

        for (int x = 0; x < length; x += length - 1)
            for (int y = 0; y < height; ++y)
                for (int z = 0; z < width; ++z)
                    grid.set({x, y, z}, {true});

        for (int x = 0; x < length; ++x)
            for (int y = 0; y < height; ++y)
                for (int z = 0; z < width; z += width - 1)
                    grid.set({x, y, z}, {true});
    }

    std::vector<BoundingBox> extractPrimitives(VoxelGrid<VoxelState> &grid) override
    {
        std::unordered_set<VoxelCoords> visited;

        const auto gridHashMap = grid.getHashMap();

        std::vector<BoundingBox> parallelepipeds;

        for (auto it : gridHashMap) {
            const auto &coord = it.first;
            const auto &voxel = it.second;
            if (!voxel.solid || visited.count(coord)) {
                // already processed this voxel
                continue;
            }

            visited.emplace(coord);

            BoundingBox bbox{coord, coord};
            std::vector<VoxelCoords> expansion;

            // try to expand the parallelepiped in every direction as far as we can
            for (auto direction : directions) {
                for (int sign = -1; sign <= 1; sign += 2) {
                    auto d = direction * sign;

                    bool canExpand = true;

                    // expanding in a specific direction as far as we can
                    while (true) {
                        const auto xlim = startEndCoord(bbox.min.x(), bbox.max.x(), d.x());
                        const auto ylim = startEndCoord(bbox.min.y(), bbox.max.y(), d.y());
                        const auto zlim = startEndCoord(bbox.min.z(), bbox.max.z(), d.z());

                        expansion.clear();

                        for (auto x = xlim.min; x <= xlim.max; ++x)
                            for (auto y = ylim.min; y <= ylim.max; ++y)
                                for (auto z = zlim.min; z <= zlim.max; ++z) {
                                    const VoxelCoords coords{x, y, z};
                                    const auto v = grid.get(coords);
                                    if (!v || !v->solid || visited.count(coords)) {
                                        // we could not expand in this direction
                                        canExpand = false;
                                        goto afterLoop;
                                    }

                                    expansion.emplace_back(coords);
                                }

                        afterLoop:

                        if (!canExpand)
                            break;

                        for (auto newVoxelCoord : expansion) {
                            visited.emplace(newVoxelCoord);
                            bbox.addPoint(newVoxelCoord);
                        }
                    }
                }
            }

            // finished expanding in all possible directions
            // the bounding box defines the parallepiped completely filled by solid voxels
            // we can draw only this parallelepiped (8 vertices) instead of drawing individual voxels, saving a ton of time
            parallelepipeds.emplace_back(bbox);
        }

        return parallelepipeds;
    }

    BoundingBox levelExit(const VoxelGrid<VoxelState> &, int numAgents) override
    {
        const auto exitPadWidth = std::min(3, numAgents);

        // make sure exit pad will fit
        assert(width - 2 >= exitPadWidth);

        const int xCoord = randRange(length - 2, length - 1, rng);
        const int zCoord = randRange(1, width - numAgents, rng);

        const VoxelCoords minCoord{xCoord, 1, zCoord}, maxCoord{xCoord + 1, 2, zCoord + exitPadWidth};

        return {minCoord, maxCoord};
    }

    std::vector<VoxelCoords> startingPositions(const VoxelGrid<VoxelState> &, int numAgents) override
    {
        std::vector<VoxelCoords> agentPositions;

        for (int i = 0; i < numAgents; ++i) {
            for (int attempt = 0; attempt < 10; ++attempt) {
                const auto agentPos = VoxelCoords{randRange(1, length - 1, rng), 1, randRange(1, width - 1, rng)};

                if (!contains(agentPositions, agentPos)) {
                    agentPositions.emplace_back(agentPos);
                    break;
                }
            }
        }

        return agentPositions;
    }
};


class LayoutGeneratorWalls : public LayoutGeneratorBasic
{
public:
    explicit LayoutGeneratorWalls(Rng &rng)
    : LayoutGeneratorBasic(rng)
    {
    }

    void init() override
    {
        LayoutGeneratorBasic::init();
        length = randRange(10, 30, rng);

        const auto numWalls = randRange(1, 4, rng);
        maxWallHeight = maxWallX = 0;
        std::vector<int> wallXCoords;

        for (int i = 0; i < numWalls; ++i) {
            const auto wallHeight = randRange(1, 2, rng);

            for (int attempt = 0; attempt < 10; ++attempt) {
                const auto wallX = randRange(4, length - 3, rng);

                if (!contains(wallXCoords, wallX)) {
                    wallXCoords.emplace_back(wallX);
                    walls.emplace_back(std::make_pair(wallX, wallHeight));
                    maxWallHeight = std::max(maxWallHeight, wallHeight);
                    maxWallX = std::max(maxWallX, wallX);
                    break;
                }
            }

        }

        height = randRange(3, 5, rng) + maxWallHeight;
    }

    void generate(VoxelGrid<VoxelState> &grid) override
    {
        LayoutGeneratorBasic::generate(grid);

        for (const auto &wall : walls) {
            const auto wallX = wall.first;
            const auto wallHeight = wall.second;

            for (int y = 1; y < 1 + wallHeight; ++y)
                for (int z = 1; z < width - 1; ++z) {
                    VoxelCoords coord{wallX, y, z};
                    grid.set(coord, {true});
                }
        }
    }

    std::vector<VoxelCoords> startingPositions(const VoxelGrid<VoxelState> &, int numAgents) override
    {
        std::vector<VoxelCoords> agentPositions;

        for (int i = 0; i < numAgents; ++i) {
            for (int attempt = 0; attempt < 10; ++attempt) {
                const auto agentPos = VoxelCoords{randRange(1, 4, rng), 1, randRange(1, width - 1, rng)};

                if (!contains(agentPositions, agentPos)) {
                    agentPositions.emplace_back(agentPos);
                    break;
                }
            }
        }

        return agentPositions;
    }

    BoundingBox levelExit(const VoxelGrid<VoxelState> &, int numAgents) override
    {
        const auto exitPadWidth = std::min(3, numAgents);
        // make sure exit pad will fit
        assert(width - 2 >= exitPadWidth);

        const auto exitX = randRange(maxWallX + 1, length - 1, rng);
        const auto exitZ = randRange(1, width - 1 - exitPadWidth, rng);

        VoxelCoords minCoord{exitX, 1, exitZ}, maxCoord{exitX + 1, 2, exitZ + exitPadWidth};
        return {minCoord, maxCoord};
    }

private:
    int maxWallHeight{}, maxWallX{};
    std::vector<std::pair<int, int>> walls;
};


class LayoutGeneratorCave : public LayoutGeneratorBasic
{
public:
    explicit LayoutGeneratorCave(Rng &rng)
    : LayoutGeneratorBasic(rng)
    {
    }

    void init() override
    {
        LayoutGeneratorBasic::init();

        caveHeight = randRange(2, 5, rng);
        height = randRange(3, 5, rng) + caveHeight;
    }

    void generate(VoxelGrid<VoxelState> &grid) override
    {
        LayoutGeneratorBasic::generate(grid);

        // generate the actual cavity

        auto growthProb = 0.8f;

        std::queue<VoxelCoords> q;
        std::unordered_set<VoxelCoords> cave;

        auto numSeeds = std::max(1, std::max(length, width) / 7 + 1);

        for (auto seed = 0; seed < numSeeds; ++seed) {
            auto seedX = randRange(2, length - 2, rng);
            auto seedZ = randRange(2, width - 2, rng);

            auto initial = VoxelCoords {seedX, caveHeight, seedZ};
            cave.emplace(initial);
            q.emplace(initial);
        }

        while (!q.empty()) {
            auto curr = q.front();
            q.pop();

            for (auto direction : directions)
                for (int sign = -1; sign <= 1; sign += 2) {
                    auto d = sign * direction;
                    VoxelCoords newCoords{curr.x() + d.x(), curr.y() + d.y(), curr.z() + d.z()};

                    auto p = frand(rng);
                    if (p > growthProb)
                        continue;

                    if (cave.count(newCoords))
                        continue;

                    if (newCoords.y() > caveHeight || newCoords.y() < 1)
                        continue;

                    if (newCoords.x() >= length - 2 || newCoords.x() < 2)
                        continue;

                    if (newCoords.z() > width - 1 || newCoords.z() < 1)
                        continue;

                    q.emplace(newCoords);
                    cave.emplace(newCoords);
                    growthProb *= 0.995f;
                }
        }

        // generate the top surface

        for (int x = 1; x < length; ++x)
            for (int z = 1; z < width; ++z) {
                VoxelCoords coords{x, caveHeight, z};
                if (cave.count(coords))
                    continue;

                grid.set(coords, {true});
            }

        // generate the walls of the cave

        for (const auto &coord : cave) {
            for (auto direction : directions)
                for (int sign = -1; sign <= 1; sign += 2) {
                    auto d = direction * sign;
                    VoxelCoords adjacent{coord.x() + d.x(), coord.y() + d.y(), coord.z() + d.z()};
                    if (adjacent.y() > caveHeight)
                        continue;

                    if (cave.count(adjacent))
                        continue;

                    grid.set(adjacent, {true});
                }
        }

        // starting positions candidates
        freeVoxels = getFreeVoxels(grid, length, width, caveHeight + 1);
        std::shuffle(freeVoxels.begin(), freeVoxels.end(), rng);
    }

    std::vector<VoxelCoords> startingPositions(const VoxelGrid<VoxelState> &, int numAgents) override
    {
        return std::vector<VoxelCoords>(freeVoxels.begin(), freeVoxels.begin() + numAgents);
    }

    BoundingBox levelExit(const VoxelGrid<VoxelState> &grid, int numAgents) override
    {
        const auto exitPadWidth = std::min(3, numAgents);
        // make sure exit pad will fit
        assert(width - 2 >= exitPadWidth);

        VoxelCoords minCoord{1, 1, 1}, maxCoord{2, 2, 2};

        for (int i = int(freeVoxels.size()) - 1; i >= 0; --i) {
            const auto &v = freeVoxels[i];

            bool fits = true;
            for (int z = v.z(); z < v.z() + exitPadWidth; ++z) {
                const auto voxel = grid.get({v.x(), v.y(), z});
                if (voxel && voxel->solid) {
                    fits = false;
                    break;
                }
            }

            if (!fits) {
                // can't put exit pad here, try elsewhere
                continue;
            }

            minCoord = v;
            maxCoord = VoxelCoords{v.x() + 1, v.y() + 1, v.z() + exitPadWidth};
            break;
        }

        return {minCoord, maxCoord};
    }

private:
    int caveHeight{};

    std::vector<VoxelCoords> freeVoxels;
};


// Wrapper class

LayoutGenerator::LayoutGenerator(Rng &rng)
    : rng{rng}
{

}


LayoutGenerator::~LayoutGenerator() = default;


void LayoutGenerator::init(LayoutType layoutType)
{
    switch (layoutType) {
        case LayoutType::Empty:
            generator = std::make_unique<LayoutGeneratorBasic>(rng);
            break;
        case LayoutType::Walls:
            generator = std::make_unique<LayoutGeneratorWalls>(rng);
            break;
        case LayoutType::Cave:
            generator = std::make_unique<LayoutGeneratorCave>(rng);
            break;
        default:
            TLOG(ERROR) << "Layout type not supported " << int(layoutType);
            break;
    }

    generator->init();
}


void LayoutGenerator::generate(VoxelGrid<VoxelState> &grid)
{
    generator->generate(grid);
}

std::vector<BoundingBox> LayoutGenerator::extractPrimitives(VoxelGrid<VoxelState> &grid)
{
    return generator->extractPrimitives(grid);
}

BoundingBox LayoutGenerator::levelExit(const VoxelGrid<VoxelState> &grid, int numAgents)
{
    return generator->levelExit(grid, numAgents);
}

std::vector<VoxelCoords> LayoutGenerator::startingPositions(const VoxelGrid<VoxelState> &grid, int numAgents)
{
    return generator->startingPositions(grid, numAgents);
}
