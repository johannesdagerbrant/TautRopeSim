#pragma once

#include "CoreMinimal.h"

namespace TautRope
{
	struct FHitData;

	struct TAUTROPESIM_API FPoint
	{
		FPoint(const FVector& InLocation);
		FPoint(const FHitData& HitData);
		FVector Location = FVector::ZeroVector;
		int32 ShapeIndex = INDEX_NONE;
		int32 EdgeIndex = INDEX_NONE;
		int32 VertIndex = INDEX_NONE;
	};
}