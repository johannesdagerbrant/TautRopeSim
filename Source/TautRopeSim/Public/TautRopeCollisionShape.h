#pragma once

#include "CoreMinimal.h"
#include "TautRopeConfig.h"

struct FKConvexElem;

namespace TautRope
{
	struct TAUTROPESIM_API FRopeCollisionShape
	{
		FRopeCollisionShape(const FKConvexElem& Convex, const FTransform& CompTransform);

		TArray<FVector> Vertices;
		TArray<FIntVector2>	Edges;
		TArray<TArray<int32>> VertToEdges;
		TArray<FQuat> EdgeRotations;

	private:
		int32 FindOrAddVertex(FVector Vertex);
		int32 AddUniqueEdge(int32 V1, int32 V2);
		int32 AddUniqueTriangle(int32 V1, int32 V2, int32 V3, TArray<FIntVector>& InOutTriangles) const;

#if TAUT_ROPE_DEBUG_DRAWING
	public:
		void DrawDebug(const UWorld* World) const;
#endif // TAUT_ROPE_DEBUG_DRAWING
	};
}