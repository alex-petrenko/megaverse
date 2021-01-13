#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cone.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Icosphere.h>

#include <rendering/render_utils.hpp>

using namespace Magnum;
using namespace VoxelWorld;

void VoxelWorld::initPrimitives(std::map<DrawableType, Magnum::Trade::MeshData> &meshes)
{
    auto &m = meshes;  // type less

    m.emplace(DrawableType::Box, Primitives::cubeSolid());
    m.emplace(DrawableType::Capsule, Primitives::capsule3DSolid(3, 3, 8, 1.0));
    m.emplace(DrawableType::Sphere, Primitives::icosphereSolid(1));
    m.emplace(DrawableType::Cone, Primitives::coneSolid(1, 6, 0.5f));
    m.emplace(DrawableType::Cylinder, Primitives::cylinderSolid(1, 6, 0.5f, Magnum::Primitives::CylinderFlag::CapEnds));
}
