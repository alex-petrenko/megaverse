#pragma once

#include <Magnum/Math/Vector.h>
#include <Magnum/Math/Color.h>
#include <Magnum/SceneGraph/SceneGraph.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>


namespace Megaverse
{

using Object3D = Magnum::SceneGraph::Object<Magnum::SceneGraph::MatrixTransformation3D>;
using Scene3D = Magnum::SceneGraph::Scene<Magnum::SceneGraph::MatrixTransformation3D>;

using Radians = Magnum::Math::Rad<Magnum::Float>;


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

inline Magnum::Math::Deg<Magnum::Float> degrees(double value) { return Magnum::Math::Deg<Magnum::Float>(Magnum::Float(value)); }

}

namespace Magnum::Math
{

template<std::size_t size, class T> inline Vector<size, int> lround(const Vector<size, T>& a)
{
    Vector<size, int> out{Magnum::NoInit};
    for(std::size_t i = 0; i != size; ++i)
        out[i] = std::lround(a[i]);
    return out;
}

template<typename CONTAINER_T>
std::vector<Magnum::Vector3> toFloat(const CONTAINER_T &vectors)
{
    std::vector<Magnum::Vector3> result;
    result.reserve(vectors.size());

    for (const auto &v : vectors)
        result.emplace_back(v);

    return result;
}

}