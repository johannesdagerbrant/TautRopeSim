#pragma once

#include "CoreMinimal.h"
#include "TautRopeCollisionShape.h"

namespace TautRope
{

	struct TAUTROPESIM_API FHitData
	{
		bool bIsHit = false;
		bool bIsHitOnFirstTriangleSweep = false;
		int32 RopePointIndex = INDEX_NONE;
		FVector Location = FVector::ZeroVector;
		int32 ShapeIndex = INDEX_NONE;
		int32 EdgeIndex = INDEX_NONE;
		FVector BaryCoords = FVector(MAX_FLT);
	};

	struct FPoint;

	void SweepSegmentThroughShapes(
		FHitData& OutHitData
		, FPoint& InOutSegmentPointA
		, FPoint& InOutSegmentPointB
		, const FVector& OriginLocationA
		, const FVector& OriginLocationB
		, const FVector& TargetLocationA
		, const FVector& TargetLocationB
		, const TArray<FRopeCollisionShape>& Shapes
		, const int32 RopePointIndex
	);
	void SweepTriangleAgainstShape(
		const FVector& TriA,
		const FVector& TriB,
		const FVector& TriC,
		const FRopeCollisionShape& Shape,
		const int32 ShapeIndex,
		const int32 ShapeIndexPointA,
		const int32 ShapeIndexPointB,
		const int32 EdgeIndexPointA,
		const int32 EdgeIndexPointB,
		const int32 VertIndexPointA,
		const int32 VertIndexPointB,
		const int32 RopePointIndex,
		const bool bIsFirstTriangleSweep,
		FHitData& OutHitData
	);
	bool GetTriangleLineIntersection(
		const FVector& TriA
		, const FVector& TriB
		, const FVector& TriC
		, const FVector& LineA
		, const FVector& LineB
		, FVector& OutLocation
		, FVector& OutBaryCoords
	);
#if TAUT_ROPE_DEBUG_DRAWING
	void DebugDrawSweep(
		const UWorld* World
		, const FVector& OriginLocationA
		, const FVector& OriginLocationB
		, const FVector& TargetLocationA
		, const FVector& TargetLocationB
		, const FHitData& OutHitData
	);
#endif // TAUT_ROPE_DEBUG_DRAWING
}