#include <queue>

#include <util/magnum.hpp>
#include <util/tiny_logger.hpp>

#include <scenarios/layout_utils.hpp>
#include <scenarios/component_voxel_grid.hpp>


using namespace Magnum;
using namespace VoxelWorld;


// TODO: add different types of layouts
void VoxelWorld::addBoundingBoxes(DrawablesMap &drawables, Env::EnvState &envState, const Boxes &boxes, int voxelType, float voxelSize)
{
    if (voxelType == VOXEL_EMPTY)
        return;

    for (auto box : boxes) {
        const auto bboxMin = box.min, bboxMax = box.max;
        auto scale = Magnum::Vector3{
            float(bboxMax.x() - bboxMin.x() + 1) / 2,
            float(bboxMax.y() - bboxMin.y() + 1) / 2,
            float(bboxMax.z() - bboxMin.z() + 1) / 2,
        } * voxelSize;

        auto translation = Magnum::Vector3{
            float((bboxMin.x() + bboxMax.x())) / 2 + 0.5f,
            float((bboxMin.y() + bboxMax.y())) / 2 + 0.5f,
            float((bboxMin.z() + bboxMax.z())) / 2 + 0.5f
        } * voxelSize;

        auto &layoutBox = envState.scene->addChild<Object3D>();
        layoutBox.scale(scale).translate(translation);

        if (voxelType & VOXEL_OPAQUE)
            drawables[DrawableType::Box].emplace_back(&layoutBox, rgb(ColorRgb::LAYOUT)); // TODO support multiple layout colors

        if (voxelType & VOXEL_SOLID) {
            auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1,1,1});
            auto &collisionBox = layoutBox.addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);

            collisionBox.syncPose();

            envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));
        }
    }
}

void VoxelWorld::addTerrain(DrawablesMap &drawables, Env::EnvState &envState, TerrainType type, const BoundingBox &bb, float voxelSize)
{
    const auto scale = Vector3(bb.max.x() - bb.min.x(), 1.0, bb.max.z() - bb.min.z()) * voxelSize;

    if (scale.x() > 0) {
        // otherwise we don't draw anything
        const auto pos = Magnum::Vector3(bb.min.x() * voxelSize + scale.x() / 2, bb.min.y() * voxelSize, bb.min.z() * voxelSize + scale.z() / 2);

        auto &terrainObject = envState.scene->addChild<Object3D>(envState.scene.get());
        terrainObject.scale({0.5, 0.025, 0.5}).scale(scale);
        terrainObject.translate({0.0, 0.025, 0.0});
        terrainObject.translate(pos);

        drawables[DrawableType::Box].emplace_back(&terrainObject, rgb(terrainColor(type)));
    }
}

void VoxelWorld::addStaticCollidingBox(
    DrawablesMap &drawables, Env::EnvState &envState,
    Vector3 scale, Vector3 translation, ColorRgb color
)
{
    auto &layoutBox = envState.scene->addChild<Object3D>();
    layoutBox.scale(scale).translate(translation);
    drawables[DrawableType::Box].emplace_back(&layoutBox, rgb(color));

    auto bBoxShape = std::make_unique<btBoxShape>(btVector3{1, 1, 1});
    auto &collisionBox = layoutBox.addChild<RigidBody>(envState.scene.get(), 0.0f, bBoxShape.get(), envState.physics.bWorld);
    collisionBox.syncPose();
    envState.physics.collisionShapes.emplace_back(std::move(bBoxShape));
}

Object3D * VoxelWorld::addCylinder(DrawablesMap &drawables, Object3D &parent, Magnum::Vector3 translation, Magnum::Vector3 scale, ColorRgb color)
{
    auto &rootObject = parent.addChild<Object3D>();
    rootObject.scale(scale).translate(translation);
    drawables[DrawableType::Cylinder].emplace_back(&rootObject, rgb(color));

    return &rootObject;
}

Object3D * VoxelWorld::addPillar(DrawablesMap &drawables, Object3D &parent, Magnum::Vector3 translation, Magnum::Vector3 scale, ColorRgb color)
{
    auto rootObject = addCylinder(drawables, parent, translation, scale, color);

    auto capScale = Vector3 {scale.x() * 1.2f, 0.15f, scale.z() * 1.2f};
    auto capTranslation = Vector3 {0, 0.47, 0} * scale;

    addCylinder(drawables, parent, translation + capTranslation, capScale, color);
    addCylinder(drawables, parent, translation - capTranslation, capScale, color);

    return rootObject;
}
