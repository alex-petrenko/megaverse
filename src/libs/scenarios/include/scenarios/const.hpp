#pragma once

#include <env/const.hpp>


namespace VoxelWorld::Str
{
    ConstStr obstaclesMinNumPlatforms = "obstaclesMinNumPlatforms",
             obstaclesMaxNumPlatforms = "obstaclesMaxNumPlatforms",
             obstaclesMinGap = "obstaclesMinGap",
             obstaclesMaxGap = "obstaclesMaxGap",
             obstaclesMinLava = "obstaclesMinLava",
             obstaclesMaxLava = "obstaclesMaxLava",
             obstaclesMinHeight = "obstaclesMinHeight",
             obstaclesMaxHeight = "obstaclesMaxHeight",
             obstaclesAgentAtExit = "obstaclesAgentAtExit",
             obstaclesAllAgentsAtExit = "obstaclesAllAgentsAtExit",
             obstaclesExtraReward = "obstaclesExtraReward",
             obstaclesNumAllowedMaxDifficulty = "obstaclesNumAllowedMaxDifficulty",
             obstaclesAgentCarriedObjectToExit = "obstaclesAgentCarriedObjectToExit";

    ConstStr towerPickedUpObject = "towerPickedUpObject",
             towerVisitedBuildingZoneWithObject = "towerVisitedBuildingZoneWithObject",
             towerBuildingReward = "towerBuildingReward";

    ConstStr collectSingleGood = "collectSingleGood",
             collectSingleBad = "collectSingleBad",
             collectAll = "collectAll",
             collectAbyss = "collectAbyss";

    ConstStr sokobanBoxOnTarget = "sokobanBoxOnTarget",
             sokobanBoxLeavesTarget = "sokobanBoxLeavesTarget",
             sokobanAllBoxesOnTarget = "sokobanAllBoxesOnTarget";

    ConstStr boxagoneTouchedFloor = "boxagoneTouchedFloor",
             boxagonePerStepReward = "boxagonePerStepReward";

    ConstStr exploreSolved = "exploreSolved";

    ConstStr memoryCollectGood = "memoryCollectGood",
             memoryCollectBad = "memoryCollectBad";

    ConstStr rearrangeOneMoreObjectCorrectPosition = "rearrangeOneMoreObjectCorrectPosition",
             rearrangeAllObjectsCorrectPosition = "rearrangeAllObjectsCorrectPosition";
}
