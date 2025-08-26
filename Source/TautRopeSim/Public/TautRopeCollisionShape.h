#pragma once

#include "CoreMinimal.h"

struct FKConvexElem;

namespace TautRope
{
	struct TAUTROPESIM_API FCollisionShape
	{
		FCollisionShape(const FKConvexElem& Convex, const FTransform& CompTransform);

		TArray<FVector> Vertices;
		TArray<FIntVector2>	Edges;
		TArray<TArray<int32>> VertToEdges;
		TArray<FIntVector>	Triangles;
		TArray<FQuat> EdgeRotations;

	private:
		int32 FindOrAddVertex(FVector Vertex);
		int32 AddUniqueEdge(int32 V1, int32 V2);
		int32 AddUniqueTriangle(int32 V1, int32 V2, int32 V3);
	};
}