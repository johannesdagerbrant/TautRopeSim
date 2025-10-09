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
		float SweepRatio = MAX_FLT;
	};

	struct FPoint;

	void SweepRemovePoint
	(
		TArray<FPoint>& RopePoints
		, const int32 RemovePointIndex
		, const TArray<FRopeCollisionShape>& Shapes
#if TAUT_ROPE_DEBUG_DRAWING
		, const UWorld* World = nullptr
		, const bool bIsDebugDrawingActive = false
#endif
	);

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

	void SweepSegmentTriangleAgainstShape(
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
	void SweepTriangleAgainstShape(
		const FVector& FromCorner,
		const FVector& ToCorner,
		const FVector& SupportCorner,
		const FRopeCollisionShape& Shape,
		const int32 ShapeIndex,
		FHitData& OutHitData
	);
	bool GetTriangleLineIntersection(
		const FVector& TriA
		, const FVector& TriB
		, const FVector& TriC
		, const FVector& LineA
		, const FVector& LineB
		, FVector& OutLocation
		, float& OutRatioAB
	);

#if TAUT_ROPE_DEBUG_DRAWING
	void DebugDrawSegmentSweep(
		const UWorld* World
		, const FVector& OriginLocationA
		, const FVector& OriginLocationB
		, const FVector& TargetLocationA
		, const FVector& TargetLocationB
		, const FHitData& OutHitData
	);

	void DebugDrawPruningSweep(
		const UWorld* World
		, const FVector& A
		, const FVector& B
		, const FVector& C
		, bool bIsHit
	);
#endif // TAUT_ROPE_DEBUG_DRAWING
}