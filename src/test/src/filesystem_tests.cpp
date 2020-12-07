#include <gtest/gtest.h>

#include <util/tiny_logger.hpp>
#include <util/filesystem_utils.hpp>


namespace VoxelWorld
{

TEST(filesystem, pathJoin)
{
    const std::string t1{"abc"}, t2{"def"}, t3{"ghi"};
    const auto p1{pathJoin(t1)}, p2{pathJoin(t1, t2)}, p3{pathJoin(t1, t2, t3)};
    EXPECT_EQ(p1, t1);
    EXPECT_EQ(p2, t1 + pathDelim() + t2);
    EXPECT_EQ(p3, t1 + pathDelim() + t2 + pathDelim() + t3);
}

}