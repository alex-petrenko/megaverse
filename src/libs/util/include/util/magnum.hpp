#pragma once

#include <Magnum/Math/Color.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>


namespace VoxelWorld
{

typedef Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D> Object3D;
typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;


template<typename T>
std::ostream &operator<<(std::ostream &stream, const Magnum::Math::Vector3<T> &v)
{
    stream << v.x() << " " << v.y() << " " << v.z();
    return stream;
}

inline Magnum::Color3 toRgbf(unsigned long long value)
{
    return Magnum::Math::unpack<Magnum::Color3>(
        Magnum::Math::Color3<Magnum::UnsignedByte>{
            Magnum::UnsignedByte(value >> 16), Magnum::UnsignedByte(value >> 8), Magnum::UnsignedByte(value)
        }
    );
}

}