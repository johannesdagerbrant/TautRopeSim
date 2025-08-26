#pragma once

#include "CoreMinimal.h"
#include "TautRopeCollisionShape.h"

namespace TautRope
{
	struct TAUTROPESIM_API FSlideOntoEdgeData
	{
		int32 EdgeIndex = INDEX_NONE;
		float RopeSlideDot = 0.f;
		bool operator<(const FSlideOntoEdgeData& Other) const
		{
			return RopeSlideDot < Other.RopeSlideDot;
		}
	};
}