#pragma once

#include <util/util.hpp>
#include <util/magnum.hpp>


namespace VoxelWorld
{

using ConstStr = const char *const;

namespace Str
{
    ConstStr teamSpirit = "teamSpirit";

    ConstStr episodeLengthSec = "episodeLengthSec",
             verticalLookLimitRad = "verticalLookLimitRad",
             useUIRewardIndicators = "useUIRewardIndicators";
}


/**
 * Enum with default colors
 */
enum class ColorRgb
{
    YELLOW = 0xffdd3c,
    GREEN = 0x3bb372,
    LIGHT_GREEN = 0x50c878,
    BLUE = 0x2eb5d0,
    LIGHT_BLUE = 0xadd8e6,
    DARK_BLUE = 0x3a7fa6,
    DARK_NAVY = 0x2c3e50,
    ORANGE = 0xffb400,
    GREY = 0xb3b3b3,
    DARK_GREY = 0x555555,
    VERY_DARK_GREY = 0x222222,
    WHITE = 0xffffff,
    RED = 0xff0000,
    LIGHT_ORANGE = 0xffa770,
    VIOLET = 0xd468ee,
    LIGHT_PINK = 0xffe6e6,

    VERY_LIGHT_YELLOW = 0xffffe6,
    VERY_LIGHT_GREEN = 0xccffcc,
    VERY_LIGHT_BLUE = 0xe6ecff,
    VERY_LIGHT_GREY = 0xd9d9d9,
    VERY_LIGHT_VIOLET = 0xf2e6ff,
    VERY_LIGHT_ORANGE = 0xffebcc,

    LAYOUT_DEFAULT = WHITE,
    AGENT_EYES = DARK_NAVY,
    MOVABLE_BOX = LIGHT_BLUE,
    EXIT_PAD = LIGHT_GREEN,
    BUILDING_ZONE = DARK_GREY,
};

const ColorRgb allColors[] = {
    ColorRgb::YELLOW,
    ColorRgb::GREEN,
    ColorRgb::LIGHT_GREEN,
    ColorRgb::BLUE,
    ColorRgb::LIGHT_BLUE,
    ColorRgb::DARK_BLUE,
    ColorRgb::DARK_NAVY,
    ColorRgb::ORANGE,
    ColorRgb::GREY,
    ColorRgb::DARK_GREY,
    ColorRgb::VERY_DARK_GREY,
    ColorRgb::WHITE,
    ColorRgb::RED,
    ColorRgb::LIGHT_ORANGE,
    ColorRgb::VIOLET,
    ColorRgb::LIGHT_PINK,

    ColorRgb::VERY_LIGHT_YELLOW,
    ColorRgb::VERY_LIGHT_GREEN,
    ColorRgb::VERY_LIGHT_BLUE,
    ColorRgb::VERY_LIGHT_GREY,
    ColorRgb::VERY_LIGHT_VIOLET,
    ColorRgb::VERY_LIGHT_ORANGE,
};
const int numColors = ARR_LENGTH(allColors);

const ColorRgb agentColors[] = {ColorRgb::YELLOW, ColorRgb::GREEN, ColorRgb::BLUE, ColorRgb::ORANGE, ColorRgb::VIOLET, ColorRgb::VERY_DARK_GREY, ColorRgb::RED};
const int numAgentColors = ARR_LENGTH(agentColors);

inline Magnum::Color3 rgb(ColorRgb color) { return toRgbf((unsigned long long)color); }

inline ColorRgb sampleRandomColor(Rng &rng)
{
    const auto idx = randRange(0, numColors, rng);
    return allColors[idx];
}

const ColorRgb objectColors[] = {
    ColorRgb::YELLOW,
    ColorRgb::GREEN,
    ColorRgb::LIGHT_GREEN,
    ColorRgb::BLUE,
    ColorRgb::LIGHT_BLUE,
    ColorRgb::DARK_BLUE,
    ColorRgb::ORANGE,
    ColorRgb::GREY,
    ColorRgb::DARK_GREY,
    ColorRgb::WHITE,
    ColorRgb::RED,
    ColorRgb::LIGHT_ORANGE,
    ColorRgb::VIOLET,
    ColorRgb::LIGHT_PINK,
};

const int numObjectColors = ARR_LENGTH(objectColors);

inline ColorRgb randomObjectColor(Rng &rng)
{
    const auto idx = randRange(0, numObjectColors, rng);
    return objectColors[idx];
}

const ColorRgb layoutColors[] = {
    ColorRgb::LAYOUT_DEFAULT,
    ColorRgb::VERY_LIGHT_YELLOW,
    ColorRgb::VERY_LIGHT_GREEN,
    ColorRgb::VERY_LIGHT_BLUE,
    ColorRgb::VERY_LIGHT_GREY,
    ColorRgb::VERY_LIGHT_ORANGE,
    ColorRgb::GREY,
    ColorRgb::GREY,
    ColorRgb::GREY,
    ColorRgb::GREY,
    ColorRgb::DARK_GREY,
    ColorRgb::DARK_GREY,
    ColorRgb::DARK_GREY,
    ColorRgb::DARK_GREY,
};
const int numLayoutColors = ARR_LENGTH(layoutColors);

inline ColorRgb randomLayoutColor(Rng &rng)
{
    const auto idx = randRange(0, numLayoutColors, rng);
    return layoutColors[idx];
}

}
