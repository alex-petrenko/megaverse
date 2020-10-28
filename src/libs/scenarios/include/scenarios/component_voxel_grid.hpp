#pragma once

#include <unordered_set>

#include <util/voxel_grid.hpp>

#include <env/scenario_component.hpp>


namespace VoxelWorld
{


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

    void addPoint(VoxelCoords v)
    {
        if (v.x() <= min.x() && v.y() <= min.y() && v.z() <= min.z())
            min = v;
        else if (v.x() >= max.x() && v.y() >= max.y() && v.z() >= max.z())
            max = v;
    }

    void sort()
    {
        // sort the min/max vertices after the transformation
        // this only works for rotations in 90-degree increments
        if (min.x() > max.x()) std::swap(min.x(), max.x());
        if (min.y() > max.y()) std::swap(min.y(), max.y());
        if (min.z() > max.z()) std::swap(min.z(), max.z());
    }

public:
    VoxelCoords min, max;
};

struct CoordRange
{
    int min, max;
};

inline CoordRange startEndCoord(int bboxMin, int bboxMax, int direction)
{
    if (direction == 1)
        return {bboxMax + 1, bboxMax + 1};
    else if (direction == -1)
        return {bboxMin - 1, bboxMin - 1};
    else
        return {bboxMin, bboxMax};
}

using Boxes = std::vector<BoundingBox>;


/**
 * Environments that use voxel grids for layouts or runtime checks should include this component.
 * @tparam VoxelT data stored in each non-empty voxel cell.
 */
template<typename VoxelT>
class VoxelGridComponent : public ScenarioComponent
{
public:
    explicit VoxelGridComponent(Scenario &scenario, int maxVoxelsXYZ = 100, float minX = 0, float minY = 0, float minZ = 0, float voxelSize = 1)
    : ScenarioComponent{scenario}
    , grid{size_t(maxVoxelsXYZ), {minX, minY, minZ}, voxelSize}
    {
    }

    void reset(Env &, Env::EnvState &) override { grid.clear(); }

    template<typename... Args>
    void addBoundingBox(const BoundingBox &bb, Args&&... args)
    {
        for (int x = bb.min.x(); x < bb.max.x(); ++x)
            for (int y = bb.min.y(); y < bb.max.y(); ++y)
                for (int z = bb.min.z(); z < bb.max.z(); ++z)
                    grid.set({x, y, z}, VoxelT(std::forward<Args>(args)...));
    }

    template<typename... Args>
    void addTerrainBoundingBox(const BoundingBox &bb, int terrain)
    {
        for (int x = bb.min.x(); x < bb.max.x(); ++x)
            for (int y = bb.min.y(); y < bb.max.y(); ++y)
                for (int z = bb.min.z(); z < bb.max.z(); ++z) {
                    const VoxelCoords coords{x, y, z};
                    if (!grid.hasVoxel(coords))
                        grid.set({x, y, z}, VoxelT());

                    grid.get(coords)->terrain |= terrain;
                }
    }

    std::map<int, Boxes> toBoundingBoxes()
    {
        const static Magnum::Vector3i directions[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

        std::unordered_set<VoxelCoords> visited;

        const auto gridHashMap = grid.getHashMap();

        std::map<int, Boxes> boxesByVoxelType;

        for (auto it : gridHashMap) {
            const auto &coord = it.first;
            const auto &voxel = it.second;
            const auto voxelType = voxel.voxelType;

            if (visited.count(coord)) {
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
                                    if (!v || v->voxelType != voxelType || visited.count(coords)) {
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
            boxesByVoxelType[voxelType].emplace_back(bbox);
        }

        return boxesByVoxelType;
    }

public:
    VoxelGrid<VoxelT> grid;
};

}