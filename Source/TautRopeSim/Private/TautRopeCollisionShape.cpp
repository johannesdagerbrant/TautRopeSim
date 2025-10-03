#include "TautRopeCollisionShape.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"
#include "Components/PrimitiveComponent.h"

namespace TautRope
{
	FRopeCollisionShape::FRopeCollisionShape(
		const FKConvexElem& Convex
		, const FTransform& CompTransform
	)
	{
		TArray<FIntVector> Triangles;
		for (int32 TriIndex = 0; TriIndex < Convex.IndexData.Num(); TriIndex += 3)
		{
			const int32 TriVertIndexA = Convex.IndexData[TriIndex];
			const int32 TriVertIndexB = Convex.IndexData[TriIndex + 1];
			const int32 TriVertIndexC = Convex.IndexData[TriIndex + 2];

			int32 ShapeVertIndexA = FindOrAddVertex(Convex.VertexData[TriVertIndexA], Vertices);
			int32 ShapeVertIndexB = FindOrAddVertex(Convex.VertexData[TriVertIndexB], Vertices);
			int32 ShapeVertIndexC = FindOrAddVertex(Convex.VertexData[TriVertIndexC], Vertices);
			const FVector& VertA = Vertices[ShapeVertIndexA];
			const FVector& VertB = Vertices[ShapeVertIndexB];
			const FVector& VertC = Vertices[ShapeVertIndexC];
			const FVector TriCross = FVector::CrossProduct(VertB - VertA, VertC - VertA);
			const float TriArea = TriCross.Size() * 0.5f;
			if (TriArea < KINDA_SMALL_NUMBER)
			{
				continue;
			}
			AddUniqueTriangle(ShapeVertIndexA, ShapeVertIndexB, ShapeVertIndexC, Triangles);
		}

		for (FVector& Vert : Vertices)
		{
			Vert = CompTransform.TransformPosition(Vert);
		}

		TMap<int32, TArray<FIntVector>> EdgeIndexToTriangles;
		for (const FIntVector& Tri : Triangles)
		{
			const int32 EdgeIndexA = AddUniqueEdge(Tri.X, Tri.Y, Edges);
			const int32 EdgeIndexB = AddUniqueEdge(Tri.Y, Tri.Z, Edges);
			const int32 EdgeIndexC = AddUniqueEdge(Tri.Z, Tri.X, Edges);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexA).Add(Tri);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexB).Add(Tri);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexC).Add(Tri);
		}

		PopulateVertToEdges();

		EdgeRotations.SetNum(Edges.Num());
		for (auto [EdgeIndex, NeighborTriangles] : EdgeIndexToTriangles)
		{
			if (!ensure(!NeighborTriangles.IsEmpty()))
			{
				continue;
			}
			FVector TriangleNormalSum = FVector::ZeroVector;
			for (const FIntVector& NeighborTriangle : NeighborTriangles)
			{
				const FVector& A = Vertices[NeighborTriangle.X];
				const FVector& B = Vertices[NeighborTriangle.Y];
				const FVector& C = Vertices[NeighborTriangle.Z];
				TriangleNormalSum += FVector::CrossProduct(B - A, C - A).GetSafeNormal();
			}
			const FVector Forward = (Vertices[Edges[EdgeIndex].Y] - Vertices[Edges[EdgeIndex].X]).GetSafeNormal();
			const FVector Up = TriangleNormalSum.GetSafeNormal();
			EdgeRotations[EdgeIndex] = FRotationMatrix::MakeFromXZ(Forward, Up).ToQuat();
		}
	};

	void FRopeCollisionShape::PopulateVertToEdges()
	{
		VertToEdges.Reset(Vertices.Num());
		VertToEdges.SetNum(Vertices.Num());
		for (int32 VertIndex = 0; VertIndex < Vertices.Num(); ++VertIndex)
		{
			for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
			{
				const FIntVector2& Edge = Edges[EdgeIndex];
				if (VertIndex == Edge.X || VertIndex == Edge.Y)
				{
					VertToEdges[VertIndex].Add(EdgeIndex);
				}
			}
		}
	}

    FRopeCollisionShape::FRopeCollisionShape(
       const FKConvexElem& Convex,
       const UPrimitiveComponent* PrimComp,
       const TArray<UPrimitiveComponent*>& OtherPrimComps
    )
	{
		FRopeCollisionShape IntactShape(Convex, PrimComp->GetComponentTransform());
		TArray<TArray<float>> RayDistancesFromVertices;
		TBitArray<> VerticesInsideOtherShape;
		const int32 NumNewVerts = RaycastAlongIntactShapeEdges(IntactShape, OtherPrimComps, RayDistancesFromVertices, VerticesInsideOtherShape);

		if (NumNewVerts == 0 && !VerticesInsideOtherShape.Contains(true))
		{
			*this = MoveTemp(IntactShape);
			return;
		}

		Vertices = IntactShape.Vertices;
		Edges = IntactShape.Edges;
		EdgeRotations = IntactShape.EdgeRotations;

		const int32 NumIntactShapeVerts = IntactShape.Vertices.Num();
		for (int32 VertIndex = 0; VertIndex < NumIntactShapeVerts; ++VertIndex)
		{
			if (VerticesInsideOtherShape[VertIndex])
			{
				continue; // Skip vertices inside other shape
			}

			const TArray<int32>& AttachedEdges = IntactShape.VertToEdges[VertIndex];
			const TArray<float>& RayDistances = RayDistancesFromVertices[VertIndex];

			ensure(AttachedEdges.Num() == RayDistances.Num());

			for (int32 i = 0; i < AttachedEdges.Num(); ++i)
			{
				const float& RayDistance = RayDistances[i];
				if (RayDistance == FLT_MAX)
				{
					continue; // Keep edge intact
				}

				const int32 EdgeIndex = AttachedEdges[i];
				const FIntVector2& Edge = IntactShape.Edges[EdgeIndex];
				const int32 OtherVertIndex = (Edge.X == VertIndex) ? Edge.Y : Edge.X;

				const FVector& Start = IntactShape.Vertices[VertIndex];
				const FVector& End = IntactShape.Vertices[OtherVertIndex];
				const FVector NewVertPos = Start + (End - Start).GetSafeNormal() * RayDistance;
				const int32 NewVertIndex = Vertices.Add(NewVertPos);

				// Update the edge to connect to the new vertex instead of the original other vertex
				if (Edge.X == OtherVertIndex)
				{
					Edges[EdgeIndex].X = NewVertIndex;
				}
				else
				{
					Edges[EdgeIndex].Y = NewVertIndex;
				}
			}
		}
		// Remove edges where both vertices are inside other shape
		for (int32 i = Edges.Num() - 1; i >= 0; --i)
		{
			const FIntVector2& Edge = Edges[i];

			if (
				Edge.X < NumIntactShapeVerts 
				&& VerticesInsideOtherShape[Edge.X] 
				&& Edge.Y < NumIntactShapeVerts 
				&& VerticesInsideOtherShape[Edge.Y]
			)
			{
				Edges.RemoveAt(i);
				EdgeRotations.RemoveAt(i);
			}
		}
		/*
		// Remove vertices that are inside other shape
		// TODO: Make edges adjust to these vertices getting removed
		for (int32 i = NumIntactShapeVerts - 1; i >= 0; --i)
		{
			if (VerticesInsideOtherShape[i])
			{
				Vertices.RemoveAt(i);
			}
		}
		*/
		PopulateVertToEdges();
    };

	int32 FRopeCollisionShape::RaycastAlongIntactShapeEdges(
		const FRopeCollisionShape& IntactShape
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, TArray<TArray<float>>& OutRayDistancesFromVertices
		, TBitArray<>& OutVerticesInsideOtherShape
	) const
	{
		const int32 NumVerts = IntactShape.Vertices.Num();
		OutVerticesInsideOtherShape.Init(false, NumVerts);
		OutRayDistancesFromVertices.SetNum(NumVerts);
		for (int32 VertIndex = 0; VertIndex < NumVerts; ++VertIndex)
		{
			OutRayDistancesFromVertices[VertIndex].Init(FLT_MAX, IntactShape.VertToEdges[VertIndex].Num());
		}

		FHitResult LatestHitResult;
		FCollisionQueryParams TraceParams = FCollisionQueryParams();
		int32 NumNewVerts = 0;
		for (int32 VertIndex = 0; VertIndex < OutRayDistancesFromVertices.Num(); ++VertIndex)
		{
			const TArray<int32>& AttachedEdges = IntactShape.VertToEdges[VertIndex];
			TArray<float>& RayDistancesFromVertex = OutRayDistancesFromVertices[VertIndex];
			const int32 NumRayDirections = RayDistancesFromVertex.Num();
			ensure(NumRayDirections == AttachedEdges.Num());
			for (int32 RayDirectionIndex = 0; RayDirectionIndex < NumRayDirections; ++RayDirectionIndex)
			{
				float& RayDistance = RayDistancesFromVertex[RayDirectionIndex];
				const int32 EdgeIndex = AttachedEdges[RayDirectionIndex];
				const FIntVector2& Edge = IntactShape.Edges[EdgeIndex];
				const int32 OtherVertIndex = (Edge.X == VertIndex) ? Edge.Y : Edge.X;
				const FVector& Start = IntactShape.Vertices[VertIndex];
				const FVector& End = IntactShape.Vertices[OtherVertIndex];
				for (UPrimitiveComponent* OtherPrimComp : OtherPrimComps)
				{
					if (!OtherPrimComp->LineTraceComponent(LatestHitResult, Start, End, TraceParams))
					{
						continue;
					}
					if (LatestHitResult.bStartPenetrating || LatestHitResult.Distance < SMALL_NUMBER)
					{
						RayDistance = -1.f;
						break;
					}
					if (LatestHitResult.Distance < RayDistance)
					{
						RayDistance = LatestHitResult.Distance;
					}
				}
				if (RayDistance < 0.f)
				{
					OutVerticesInsideOtherShape[VertIndex] = true;
					break;
				}
				else if(RayDistance < FLT_MAX)
				{
					NumNewVerts++;
				}
			}
		}
		return NumNewVerts;
	};

	int32 FRopeCollisionShape::FindOrAddVertex(const FVector& NewVert, TArray<FVector>& InOutVerts) const
	{
		auto DistancePredicate = [&](const FVector& Target)
			{
				return [&](const FVector& V)
					{
						return FVector::DistSquared(V, Target) <= TAUT_ROPE_DISTANCE_TOLERANCE_SQUARED;
					};
			};
		int32 VertIndex = InOutVerts.IndexOfByPredicate(DistancePredicate(NewVert));
		if (VertIndex == INDEX_NONE)
		{
			return InOutVerts.Add(NewVert);
		}
		return VertIndex;
	};

	FORCEINLINE int32 FRopeCollisionShape::AddUniqueEdge(int32 V1, int32 V2, TArray<FIntVector2>& InOutEdges) const
	{
		return InOutEdges.AddUnique((V1 < V2) ? FIntVector2(V1, V2) : FIntVector2(V2, V1));
	};

	int32 FRopeCollisionShape::AddUniqueTriangle(int32 V1, int32 V2, int32 V3, TArray<FIntVector>& InOutTriangles) const
	{
		int32 ExistingIndex = INDEX_NONE;
		const FIntVector Variants[6] = {
			{V1, V2, V3}, {V1, V3, V2},
			{V2, V1, V3}, {V2, V3, V1},
			{V3, V1, V2}, {V3, V2, V1}
		};
		for (const FIntVector& T : Variants)
		{
			ExistingIndex = InOutTriangles.IndexOfByKey(T);
			if (ExistingIndex != INDEX_NONE)
			{
				return ExistingIndex;
			}
		}
		return InOutTriangles.Add(Variants[0]);
	};

#if TAUT_ROPE_DEBUG_DRAWING
	void FRopeCollisionShape::DrawDebug(const UWorld* World) const
	{
		for (const FIntVector2& Edge : Edges)
		{
			const FVector EdgeVertA = Vertices[Edge.X];
			const FVector EdgeVertB = Vertices[Edge.Y];
			DrawDebugLine(
				World
				, Vertices[Edge.X]
				, Vertices[Edge.Y]
				, FColor::Blue
			);
		}
		for (const FVector& Vert : Vertices)
		{
			DrawDebugSphere(
				World
				, Vert
				, 5.f
				, 4
				, FColor::Red
			);
		}
	};
#endif // TAUT_ROPE_DEBUG_DRAWING
}