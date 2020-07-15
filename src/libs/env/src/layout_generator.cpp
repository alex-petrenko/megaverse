#include <queue>

#include <cassert>
#include <unordered_set>

#include <util/tiny_logger.hpp>

#include <env/layout_generator.hpp>

#include <util/util.hpp>
#include <util/magnum.hpp>


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


LayoutGenerator::LayoutGenerator()
{
    // TODO!
    srand(time(nullptr));

    caveHeight = 3;

    length = randRange(10, 25);
    width = randRange(4, 25);
    height = randRange(3, 6) + caveHeight;
}


void LayoutGenerator::generateFloor(VoxelGrid<VoxelState> &grid)
{
    for (int x = 0; x < length; ++x)
        for (int z = 0; z < width; ++z)
            grid.set({x, 0, z}, {true});
}


void LayoutGenerator::generateFloorWalls(VoxelGrid<VoxelState> &grid)
{
    TLOG(INFO) << "Generating environment layout";

    generateFloor(grid);

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

void LayoutGenerator::generateCave(VoxelGrid<VoxelState> &grid)
{
    auto growthProb = 0.39f;

    auto seedX = randRange(2, length - 2);
    auto seedZ = randRange(2, width - 2);

    std::unordered_set<VoxelCoords> cave;

    auto initial = VoxelCoords {seedX, caveHeight, seedZ};
    cave.emplace(initial);

    std::queue<VoxelCoords> q;
    q.emplace(initial);

    while (!q.empty()) {
        auto curr = q.front();
        q.pop();

        for (auto direction : directions)
            for (int sign = -1; sign <= 1; sign += 2) {
                auto d = sign * direction;
                VoxelCoords newCoords{curr.x() + d.x(), curr.y() + d.y(), curr.z() + d.z()};

                auto p = rand() / double(RAND_MAX);
                if (p > growthProb)
                    continue;

                if (cave.count(newCoords))
                    continue;

                if (newCoords.y() > caveHeight || newCoords.y() < 1)
                    continue;

                if (newCoords.x() > length - 2 || newCoords.x() < 2)
                    continue;

                if (newCoords.z() > width - 2 || newCoords.z() < 2)
                    continue;

                q.emplace(newCoords);
                cave.emplace(newCoords);
            }
    }

    for (int x = 1; x < length; ++x)
        for (int z = 1; z < width; ++z) {
            VoxelCoords coords{x, caveHeight, z};
            if (cave.count(coords))
                continue;

            grid.set(coords, {true});
        }

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
}

std::vector<BoundingBox> LayoutGenerator::extractPrimitives(VoxelGrid<VoxelState> &grid)
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

BoundingBox LayoutGenerator::levelExit(int numAgents)
{
    // make sure exit pad will fit
    assert(width - 2 >= numAgents);

    const int xCoord = randRange(1, length - 1);
    const int zCoord = randRange(1, width - numAgents);

    const VoxelCoords minCoord{xCoord, caveHeight + 1, zCoord}, maxCoord{xCoord + 1, caveHeight + 2, zCoord + numAgents};

    TLOG(INFO) << minCoord << " " << maxCoord;
    return {minCoord, maxCoord};
}

std::vector<VoxelCoords> LayoutGenerator::startingPositions()
{
    std::vector<VoxelCoords> positions;

    for (int x = 1; x < length - 1; ++x)
        for (int z = 1; z < width - 1; ++z)
            positions.emplace_back(x, 1, z);

    return positions;
}
