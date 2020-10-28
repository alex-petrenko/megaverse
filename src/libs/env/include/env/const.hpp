#pragma once

#include <util/util.hpp>
#include <util/magnum.hpp>


namespace VoxelWorld
{

using ConstStr = const char *const;

namespace Str
{
    ConstStr episodeLengthSec = "episodeLengthSec",
             verticalLookLimitRad = "verticalLookLimitRad";
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
    ORANGE = 0xffa351,
    DARK_GREY = 0x555555,
    VERY_DARK_GREY = 0x222222,
    WHITE = 0xffffff,
    RED = 0xff0000,

    LAYOUT = WHITE,
    AGENT_EYES = VERY_DARK_GREY,
    MOVABLE_BOX = LIGHT_BLUE,
    EXIT_PAD = LIGHT_GREEN,
    BUILDING_ZONE = DARK_GREY,
};

// TODO: define more colors?

const ColorRgb allColors[] = {
    ColorRgb::YELLOW, ColorRgb::GREEN, ColorRgb::LIGHT_GREEN, ColorRgb::BLUE, ColorRgb::LIGHT_BLUE, ColorRgb::ORANGE,
    ColorRgb::DARK_GREY, ColorRgb::VERY_DARK_GREY, ColorRgb::WHITE,
};
const int numColors = ARR_LENGTH(allColors);

const ColorRgb agentColors[] = {ColorRgb::YELLOW, ColorRgb::GREEN, ColorRgb::BLUE, ColorRgb::ORANGE};
const int numAgentColors = ARR_LENGTH(agentColors);

inline Magnum::Color3 rgb(ColorRgb color) { return toRgbf((unsigned long long)color); }

}
