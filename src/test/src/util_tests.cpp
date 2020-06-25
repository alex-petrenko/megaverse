#include <algorithm>

#include <gtest/gtest.h>

#include <util/util.hpp>


TEST(util, memcpyStride)
{
    char buf[] = { 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1 };
    char zeros[9], ones[6];

    memcpyStride(zeros, buf, 3, 3, 0, 5);
    auto range = std::equal_range(std::begin(zeros), std::end(zeros), 0);
    EXPECT_TRUE(range.first == std::begin(zeros) && range.second == std::end(zeros));

    memcpyStride(ones, buf, 2, 3, 3, 5);
    range = std::equal_range(std::begin(ones), std::end(ones), 1);
    EXPECT_TRUE(range.first == std::begin(ones) && range.second == std::end(ones));
}
