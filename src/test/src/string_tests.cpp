#include <gtest/gtest.h>

#include <util/string_utils.hpp>


TEST(str, startsWith)
{
    EXPECT_TRUE(startsWith("abc", "a"));
    EXPECT_TRUE(startsWith("abc", "ab"));
    EXPECT_TRUE(startsWith("abc", "abc"));
    EXPECT_TRUE(startsWith("abc", ""));
    EXPECT_FALSE(startsWith("abc", "abcd"));
    EXPECT_FALSE(startsWith("abc", "b"));
    EXPECT_TRUE(startsWith("", ""));
    EXPECT_FALSE(startsWith("", "a"));
}

TEST(str, endsWith)
{
    EXPECT_TRUE(endsWith("abc", "c"));
    EXPECT_TRUE(endsWith("abc", "bc"));
    EXPECT_TRUE(endsWith("abc", "abc"));
    EXPECT_TRUE(endsWith("abc", ""));
    EXPECT_FALSE(endsWith("abc", "_abc"));
    EXPECT_FALSE(endsWith("abc", "b"));
    EXPECT_TRUE(endsWith("", ""));
    EXPECT_FALSE(endsWith("", "a"));
}
