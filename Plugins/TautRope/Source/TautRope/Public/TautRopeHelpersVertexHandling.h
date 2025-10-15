#pragma once

#include "CoreMinimal.h"
#include "TautRopeCollisionShape.h"

namespace TautRope
{
	struct FPoint;

	TBitArray<> GetAdjacentPointsOnSameVertexCone(
		const TArray<FPoint>& RopePoints
		, const TArray<FTautRopeCollisionShape>& NearbyShapes
	);

	void LetPointsOnVertexSlideOntoNewEdge(
		TArray<FPoint>& RopePoints
		, const TArray<FTautRopeCollisionShape>& NearbyShapes
	);
	
}