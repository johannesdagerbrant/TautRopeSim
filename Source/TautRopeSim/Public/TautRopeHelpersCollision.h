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
		FVector BaryCoords = FVector::ZeroVector;
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

	bool GetTriangleLineIntersection(
		const FVector& TriA
		, const FVector& TriB
		, const FVector& TriC
		, const FVector& LineA
		, const FVector& LineB
		, FHitData& OutTautRopeHitData
	);

}