#pragma once

#include <util/voxel_grid.hpp>

#include <env/scenario_component.hpp>


namespace VoxelWorld
{

/**
 * Any environment which uses voxel grids for layouts of runtime checks should include this component.
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

public:
    VoxelGrid<VoxelT> grid;
};

}