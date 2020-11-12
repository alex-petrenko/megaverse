#pragma once


namespace VoxelWorld
{

template<typename T> T triangularNumber(T n)
{
    return n * (n + 1) / 2;
}

}