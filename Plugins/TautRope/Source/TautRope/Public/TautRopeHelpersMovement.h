#pragma once

#include "CoreMinimal.h"
#include "TautRopeCollisionShape.h"

namespace TautRope
{
	struct FMovementGroup
	{
		FMovementGroup(
			const int32 InShapeIndex
			, const int32 InVertIndex
			, const int32 InFirstPointIndex
		);
		int32 ShapeIndex = INDEX_NONE;
		int32 VertIndex = INDEX_NONE;
		int32 FirstPointIndex = INDEX_NONE;
		int32 LastPointIndex = INDEX_NONE;
		TArray<int32> EdgeIndices;
	};

	struct FPoint;

	TArray<FMovementGroup> GetMovementGroups(
		const TArray<FPoint>& RopePoints
		, const TArray<FTautRopeCollisionShape>& NearbyShapes
	);

	TArray<int32> GetCandidateVerts(
		const FPoint& P,
		const FTautRopeCollisionShape& Shape
	);

	FVector FindMinDistancePointBetweenABOnLineXY(
		const FVector& A
		, const FVector& B
		, const FVector& X
		, const FVector& Y
		, float& OutDistAlongEdge
		, float& OutEdgeLength
	);
}