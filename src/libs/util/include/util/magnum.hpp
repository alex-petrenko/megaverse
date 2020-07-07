#pragma once

#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>


typedef Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D> Object3D;
typedef Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D> Scene3D;


template <typename T>
std::ostream & operator<<(std::ostream &stream, const Magnum::Math::Vector3<T> &v)
{
    stream << v.x() << " " << v.y() << " " << v.z();
    return stream;
}