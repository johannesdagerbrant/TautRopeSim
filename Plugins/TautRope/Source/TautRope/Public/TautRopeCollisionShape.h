#pragma once

#include "CoreMinimal.h"
#include "TautRopeConfig.h"

#include "TautRopeCollisionShape.generated.h"

struct FKConvexElem;

USTRUCT()
struct FTautRopeCollisionShapeVertEdges
{
	GENERATED_BODY()

	UPROPERTY()
	TArray<int32> Edges;
};


USTRUCT()
struct TAUTROPE_API FTautRopeCollisionShape
{
	GENERATED_BODY()

public:
	FTautRopeCollisionShape() = default;
	FTautRopeCollisionShape(
		const FKConvexElem& Convex
		, const FTransform& CompTransform
	);
	FTautRopeCollisionShape(
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

	UPROPERTY()
	TArray<FVector> Vertices;

	UPROPERTY()
	TArray<FIntVector2>	Edges;

	UPROPERTY()
	TArray<FTautRopeCollisionShapeVertEdges> VertToEdges;

	UPROPERTY()
	TArray<FQuat> EdgeRotations;

private:
	UPROPERTY()
	TArray<bool> IsCornerVertexList;


	void MakeInitialHitResults(
		FHitResult& InitHitResultA
		, FHitResult& InitHitResultB
		, const FVector& A
		, const FVector& B
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, const FCollisionQueryParams& TraceParams = FCollisionQueryParams()
	) const;

	void CreateIntermedateEdges(
		const FHitResult& LastHitResultA
		, const FHitResult& LastHitResultB
		, const FQuat& EdgeRotation
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, const FCollisionQueryParams& TraceParams = FCollisionQueryParams()
	);

	void PopulateVertToEdges();
	int32 FindOrAddVertex(const FVector& NewVert, TArray<FVector>& Verts) const;
	int32 AddUniqueEdge(int32 V1, int32 V2, TArray<FIntVector2>& InOutEdges) const;
	int32 AddUniqueTriangle(int32 V1, int32 V2, int32 V3, TArray<FIntVector>& InOutTriangles) const;

#if TAUT_ROPE_DEBUG_DRAWING
public:
	void DrawDebug(const UWorld* World) const;
#endif // TAUT_ROPE_DEBUG_DRAWING
};