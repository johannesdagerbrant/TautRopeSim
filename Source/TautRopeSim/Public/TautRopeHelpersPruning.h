#pragma once

#include "CoreMinimal.h"
#include "TautRopeCollisionShape.h"

namespace TautRope
{
	bool IsRopeWrappingEdge(
		const FVector& PointLocationA
		, const FVector& PointLocationB
		, const FVector& PointLocationC
		, const FQuat& EdgeRotation
	);
}