#pragma once

#include "CoreMinimal.h"

namespace TautRope
{
	struct FPoint;
	struct FRopeCollisionShape;

	TBitArray<> GetAdjacentPointsOnSameVertexCone(
		const TArray<FPoint>& RopePoints
		, const TArray<FRopeCollisionShape>& NearbyShapes
	);

	void LetPointsOnVertexSlideOntoNewEdge(
		TArray<FPoint>& RopePoints
		, const TArray<FRopeCollisionShape>& NearbyShapes
	);
	
}