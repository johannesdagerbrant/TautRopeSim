#pragma once

#include "CoreMinimal.h"
#include "TautRopeConfig.h"

struct FKConvexElem;

namespace TautRope
{
	struct TAUTROPESIM_API FRopeCollisionShape
	{
	public:
		FRopeCollisionShape(
			const FKConvexElem& Convex
			, const FTransform& CompTransform
		);
		FRopeCollisionShape(
			const FKConvexElem& Convex
			, const UPrimitiveComponent* PrimComp
			, const TArray<UPrimitiveComponent*>& OtherPrimComps
		);

		// If a vertex has more than one adjacent edge. 
		// OBS: VertexIndex is asumed to be in valid range of Vertices array.
		FORCEINLINE bool IsCornerVertex(const int32 VertexIndex) const
		{
			return IsCornerVertexList[VertexIndex];
		};

		TArray<FVector> Vertices;
		TArray<FIntVector2>	Edges;
		TArray<TArray<int32>> VertToEdges;
		TArray<FQuat> EdgeRotations;

	private:
		TBitArray<> IsCornerVertexList;

		void PopulateVertToEdges();

		void RaycastAlongIntactShapeEdges(
			const FRopeCollisionShape& IntactShape
			, const TArray<UPrimitiveComponent*>& OtherPrimComps
			, TArray<FVector2f>& OutEdgeRayDistances
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