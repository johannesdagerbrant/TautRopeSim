#pragma once

#include "CoreMinimal.h"

namespace TautRope
{
	struct FMovementGroup
	{
		int32 VertIndex = INDEX_NONE;
	};

	FVector FindMinDistancePointBetweenABOnLineXY(
		const FVector& A
		, const FVector& B
		, const FVector& X
		, const FVector& Y
		, float& OutDistAlongEdge
		, float& OutEdgeLength
	);
}