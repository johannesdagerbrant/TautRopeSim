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

#if TAUT_ROPE_DEBUG_DRAWING
	void DebugDrawPruningSweep(
		const UWorld* World
		, const FVector& LastPointLocation
		, const FVector& RemovedPointLocation
		, const FVector& NextPointLocation
	);
#endif // TAUT_ROPE_DEBUG_DRAWING
}