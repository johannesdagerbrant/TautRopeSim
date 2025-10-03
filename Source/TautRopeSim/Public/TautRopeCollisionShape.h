#pragma once

#include "CoreMinimal.h"
#include "TautRopeConfig.h"

struct FKConvexElem;

namespace TautRope
{
	struct TAUTROPESIM_API FRopeCollisionShape
	{
		FRopeCollisionShape(
			const FKConvexElem& Convex
			, const UPrimitiveComponent* PrimComp
			, const TArray<UPrimitiveComponent*>& OtherPrimComps
		);

		TArray<FVector> Vertices;
		TArray<FIntVector2>	Edges;
		TArray<TArray<int32>> VertToEdges;
		TArray<FQuat> EdgeRotations;

	private:
		FRopeCollisionShape(
			const FKConvexElem& Convex
			, const FTransform& CompTransform
		);

		void PopulateVertToEdges();

		int32 RaycastAlongIntactShapeEdges(
			const FRopeCollisionShape& IntactShape
			, const TArray<UPrimitiveComponent*>& OtherPrimComps
			, TArray<TArray<float>>& OutRayDistancesFromVertices
			, TBitArray<>& OutVerticesInsideOtherShape
		) const;

		int32 FindOrAddVertex(const FVector& NewVert, TArray<FVector>& Verts) const;
		int32 AddUniqueEdge(int32 V1, int32 V2, TArray<FIntVector2>& InOutEdges) const;
		int32 AddUniqueTriangle(int32 V1, int32 V2, int32 V3, TArray<FIntVector>& InOutTriangles) const;

#if TAUT_ROPE_DEBUG_DRAWING
	public:
		void DrawDebug(const UWorld* World) const;
#endif // TAUT_ROPE_DEBUG_DRAWING
	};
}