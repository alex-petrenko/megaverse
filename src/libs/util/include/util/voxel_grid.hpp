#pragma once

#include <unordered_map>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include <util/magnum.hpp>


namespace VoxelWorld
{

using VoxelCoords = Magnum::Vector3i;

constexpr int maxGridResolution = 1'024, logMaxGridResolution = 10, maxAbsVoxelCoord = maxGridResolution / 2;

inline VoxelCoords toVoxel(const Magnum::Vector3 &v)
{
    return lround(floor(v));
}

}

//bool operator==(const VoxelCoords &lhs, const VoxelCoords &rhs)
//{
//    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
//}

namespace std
{

template <> struct hash<VoxelWorld::VoxelCoords>
{
    std::size_t operator()(const VoxelWorld::VoxelCoords &voxel) const noexcept
    {
        constexpr auto shift = VoxelWorld::logMaxGridResolution;
        constexpr auto offset = VoxelWorld::maxAbsVoxelCoord;
        const int x = voxel.x() + offset, y = voxel.y() + offset, z = voxel.z() + offset;

        return size_t((x << (2 * shift)) + (y << shift) + z);
    }
};

}


namespace VoxelWorld
{

template<typename VoxelState> class VoxelGrid
{
public:
    using HashMap = std::unordered_map<VoxelCoords, VoxelState>;

public:
    /**
     * Ctor.
     * @param voxelCount number of voxels to reserve initially.
     * @param origin point with the lowest coords covered by the voxelGrid. Corresponds to voxel {0, 0, 0}.
     * @param voxelSize scale of one voxel
     */
    explicit VoxelGrid(size_t voxelCount, const Magnum::Vector3 &origin, float voxelSize)
        : voxelCount{voxelCount}
          , grid{voxelCount}
          , origin{origin}
          , voxelSize{voxelSize} {}

    /**
     * Reset the grid (empty).
     */
    void clear()
    {
        grid = HashMap{voxelCount};
    }

    /**
     * @param coords coordinates of a voxel.
     * @return check whether voxel at these coordinates is present in the map.
     */
    bool hasVoxel(const VoxelCoords &coords) const
    {
        return bool(grid.count(coords));
    }

    /**
     * @param coords location in voxel grid.
     * @return pointer to VoxelState at the "coords" location, or nullptr if nothing is there.
     */
    const VoxelState *get(const VoxelCoords &coords) const
    {
        auto voxelIt = grid.find(coords);
        if (voxelIt == grid.end())
            return nullptr;

        return &(voxelIt->second);
    }

    VoxelState *get(const VoxelCoords &coords)
    {
        auto voxelIt = grid.find(coords);
        if (voxelIt == grid.end())
            return nullptr;

        return &(voxelIt->second);
    }

    /**
     * Override the voxel state in the particular voxel.
     * @param coords
     * @param state
     */
    void set(const VoxelCoords &coords, const VoxelState &state)
    {
        grid[coords] = state;
    }

    void remove(const VoxelCoords &coords)
    {
        grid.erase(coords);
    }

    /**
     * Convert floating point coordinates of a point in space to voxel coordinates.
     * @param v point (vector) in 3D space.
     * @return corresponding voxel coords.
     */
    VoxelCoords getCoords(const Magnum::Vector3 &v)
    {
        const auto coordsFloat = (v - origin) / voxelSize;
        const auto coords = toVoxel(coordsFloat);
        return coords;
    }

    const HashMap & getHashMap() const
    {
        return grid;
    }

private:
    size_t voxelCount;

    HashMap grid;

    Magnum::Vector3 origin;
    float voxelSize;
};

}