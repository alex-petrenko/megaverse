#include <cassert>
#include <unordered_set>

#include <util/tiny_logger.hpp>

#include <env/layout_generator.hpp>


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
