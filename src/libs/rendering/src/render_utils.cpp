#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cone.h>
#include <Magnum/Primitives/Capsule.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/Icosphere.h>

#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/SceneGraph/Camera.h>

#include <env/env_renderer.hpp>

#include <rendering/render_utils.hpp>

using namespace VoxelWorld;

using namespace Magnum;
using namespace Magnum::Math::Literals;


void VoxelWorld::initPrimitives(std::map<DrawableType, Magnum::Trade::MeshData> &meshes)
{
    auto &m = meshes;  // type less

    m.emplace(DrawableType::Box, Primitives::cubeSolid());
    m.emplace(DrawableType::Capsule, Primitives::capsule3DSolid(3, 3, 8, 1.0));
    m.emplace(DrawableType::Sphere, Primitives::icosphereSolid(1));
    m.emplace(DrawableType::Cone, Primitives::coneSolid(1, 6, 0.5f));
    m.emplace(DrawableType::Cylinder, Primitives::cylinderSolid(1, 6, 0.5f, Magnum::Primitives::CylinderFlag::CapEnds));
}


void Overview::reset(Object3D *parent)
{
    root = &parent->addChild<Object3D>();
    root->rotateYLocal(225.0_degf);
    root->translateLocal(Magnum::Vector3{0.1f, 20.0f, 0.1f});

    verticalTilt = &root->addChild<Object3D>();
    verticalTilt->rotateXLocal(-40.0_degf);
    verticalRotation = -40.0f;

    camera = &(verticalTilt->addFeature<SceneGraph::Camera3D>());

    auto [fov, near, far, aspectRatio] = overviewCameraParameters();
    camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
           .setProjectionMatrix(Matrix4::perspectiveProjection(Deg(fov), aspectRatio, near, far))
           .setViewport(GL::defaultFramebuffer.viewport().size());

    if (rootTransformation != Matrix4{} || verticalTiltTransformation != Matrix4{})
        restoreTransformation();
    else
        saveTransformation();
}
