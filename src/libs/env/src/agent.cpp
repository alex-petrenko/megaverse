#include <memory>

#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/GL/DefaultFramebuffer.h>

#include <env/agent.hpp>
#include <util/tiny_logger.hpp>
#include <util/util.hpp>


using namespace Magnum;
using namespace Magnum::Math::Literals;


constexpr float maxVelocity = 1.0f;


Agent::Agent(Object3D *parent) : Object(parent)
{
    cameraObject = std::make_unique<Object3D>(this);
    cameraObject->rotateY(0.0_degf);
    cameraObject->translate(Magnum::Vector3{0, 0.25f, 0});
    camera = std::make_unique<SceneGraph::Camera3D>(*cameraObject);

    camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix4::perspectiveProjection(75.0_degf, 4.0f/3.0f, 0.1f, 50.0f))
        .setViewport(GL::defaultFramebuffer.viewport().size());
}

//Magnum::Vector3 calcNewPosition(const Agent &a, const Vector3 &acceleration)
//{
//    auto newVelocity = a.velocity + acceleration;
//    auto velocityNorm = newVelocity.length();
//    if (velocityNorm > maxVelocity)
//        newVelocity *= maxVelocity / velocityNorm;
//
//
//
//    return Magnum::Vector3();
//}

void ensureAgentNotOnVoxelBoundary(Agent &a, const VoxelCoords &v)
{
    // make sure we're well within the current voxel and not right on the boundary
    auto at = a.transformation().translation();
    auto minX = v.x() + 0.01f, maxX = v.x() + 0.99f;
    auto minZ = v.z() + 0.01f, maxZ = v.z() + 0.99f;

    if (at.x() < minX)
        a.translate(Vector3{minX - at.x(), 0, 0});
    else if (at.x() > maxX)
        a.translate(Vector3{maxX - at.x(), 0, 0});

    if (at.z() < minZ)
        a.translate(Vector3{0, 0, minZ - at.z()});
    else if (at.z() > maxZ)
        a.translate(Vector3{0, 0, maxZ - at.z()});

//    auto x = std::min(v.x() + 0.99f, at.x());
//    x = std::max(v.x() + 0.01f, x);
//
//    auto z = std::min(v.z() + 0.99f, at.z());
//    z = std::max(v.z() + 0.01f, z);
//
//    a.transformation().translation().x() = 2;
//    a.transformation().translation().z() = 3;
}

void moveAgent(Agent &a, const Vector3 &delta, const VoxelGrid<VoxelState> &vg, int depth)
{
    constexpr auto eps = 1e-5f;
    if (delta.length() < eps)
        return;

    // handle falling into a pit
//    const auto yCoord = a.transformation().translation().y();
//    TLOG(INFO) << yCoord;

    // check for collisions with solid voxels
    // we can intersect a voxel in xy or yz plane

    auto at = a.transformation().translation();
    auto voxelCoords = VoxelCoords(Magnum::Math::floor(at));

    // how much of the "delta" we need to step in x/z direction to intersect a voxel boundary
    // if either of those is <1 then we're about to cross to a different voxel
    float xDelta = 1e9, zDelta = 1e9;

    if (fabs(delta.x()) > eps) {
        // change in x-direction is non-zero
        if (delta.x() > 0)
            xDelta = (ceilf(at.x()) - at.x()) / delta.x();
        else
            xDelta = (floorf(at.x()) - at.x()) / delta.x();
    }

    if (fabs(delta.z()) > eps) {
        // change in z-direction is non-zero
        if (delta.z() > 0)
            zDelta = (ceilf(at.z()) - at.z()) / delta.z();
        else
            zDelta = (floorf(at.z()) - at.z()) / delta.z();
    }

    auto adjustedDelta = delta;

    if (xDelta > 1.0f && zDelta > 1.0f) {
        // moving along the direction we're not crossing the voxel boundary
        a.translate(adjustedDelta);
        ensureAgentNotOnVoxelBoundary(a, voxelCoords);

    } else {
        if (xDelta == zDelta) {
            // rare situation, we're hitting a corner of the voxel
            // to handle it, just make sure that we hit the xy plane slightly earlier
            adjustedDelta.z() *= 1.0f - eps;
        }

        Vector3 remainingDelta;
        VoxelCoords nextVoxelCoords = voxelCoords;

        if (xDelta <= zDelta && xDelta > eps && xDelta < 1e9) {
            nextVoxelCoords.x() += sgn(adjustedDelta.x());
            const auto voxelState = vg.get(nextVoxelCoords);
            if (voxelState && voxelState->solid) {
                // we hit the wall in xy plane
                // make sure we do not cross into the occupied voxel
                auto moveAmount = xDelta * adjustedDelta;
                a.translate(moveAmount);
                ensureAgentNotOnVoxelBoundary(a, voxelCoords);

                remainingDelta = adjustedDelta - moveAmount;
                remainingDelta.x() = 0.0f;  // can't move further in x direction
            } else {
                // make sure we do cross into the next voxel
                auto moveAmount = xDelta * adjustedDelta;
                a.translate(moveAmount);
                ensureAgentNotOnVoxelBoundary(a, nextVoxelCoords);
                remainingDelta = adjustedDelta - moveAmount;
            }
        } else if (zDelta > eps && zDelta < 1e9) {
            nextVoxelCoords.z() += sgn(adjustedDelta.z());
            const auto voxelState = vg.get(nextVoxelCoords);
            if (voxelState && voxelState->solid) {
                // we hit the wall in yz plane
                // make sure we do not cross into the occupied voxel
                auto moveAmount = zDelta * adjustedDelta;
                a.translate(moveAmount);
                ensureAgentNotOnVoxelBoundary(a, voxelCoords);

                remainingDelta = adjustedDelta - moveAmount;
                remainingDelta.z() = 0.0f;  // can't move further in z direction
            } else {
                // make sure we do cross into the next voxel
                auto moveAmount = zDelta * adjustedDelta;
                a.translate(moveAmount);
                ensureAgentNotOnVoxelBoundary(a, nextVoxelCoords);

                remainingDelta = adjustedDelta - moveAmount;
            }
        }

        moveAgent(a, remainingDelta, vg, depth + 1);
    }
}
