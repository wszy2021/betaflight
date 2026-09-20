#include "follow/follow_bundle.h"



static void followOverrideComputeTargetChannel(int xOffset, int yOffset)
{
    (void)xOffset;
    (void)yOffset;

#error "Implement your private followComputeTargetChannel replacement here."
}

void followOverrideRegisterHooks(void)
{
    followSetComputeTargetChannelHook(followOverrideComputeTargetChannel);
}
