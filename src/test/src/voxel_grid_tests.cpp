#include <gtest/gtest.h>

#include <util/voxel_grid.hpp>


struct TestVoxelState
{
    int someInt;
    std::string someString;
};


TEST(voxelGrid, basic)
{
    VoxelGrid<TestVoxelState> vg{100, {0, 0, 0}, 1};

    VoxelCoords coords{1, 2, 3};
    EXPECT_EQ(coords, VoxelCoords(1, 2, 3));  // make sure operator== works
    EXPECT_EQ(std::hash<VoxelCoords>{}(coords), 1002003);

    EXPECT_EQ(vg.getCoords({1.5, 2.3, 3.2}), coords);

    auto voxelPtr = vg.get({0, 0, 0});
    EXPECT_EQ(voxelPtr, nullptr);

    TestVoxelState state{3, "42"};
    vg.set({0, 0, 0}, state);

    voxelPtr = vg.get({0, 0, 0});
    EXPECT_NE(voxelPtr, nullptr);

    const auto newStr = "I love voxels!";
    voxelPtr->someInt = 42;
    voxelPtr->someString = newStr;

    auto ptr = vg.get({0, 0, 0});
    EXPECT_EQ(voxelPtr, ptr);
    EXPECT_EQ(ptr->someInt, 42);
    EXPECT_EQ(ptr->someString, newStr);
}

TEST(voxelGrid, perf)
{
    VoxelGrid<TestVoxelState> vg{100, {0, 0, 0}, 1};

    for (int x = 0; x < 100; ++x)
        for (int y = 0; y < 100; ++y)
            for (int z = 0; z < 100; ++z)
                vg.set({x, y, z}, {x + y + z, "42"});
}