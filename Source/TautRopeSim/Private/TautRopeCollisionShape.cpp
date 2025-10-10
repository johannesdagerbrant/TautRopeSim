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

		const FTransform ConvexTransform = Convex.GetTransform() * CompTransform;
		for (FVector& Vert : Vertices)
		{
			Vert = ConvexTransform.TransformPosition(Vert);
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

		IsCornerVertexList.Init(false, Vertices.Num());
	};

    FRopeCollisionShape::FRopeCollisionShape(
       const FKConvexElem& Convex,
       const UPrimitiveComponent* PrimComp,
       const TArray<UPrimitiveComponent*>& OtherPrimComps
    )
	{
		FRopeCollisionShape IntactShape(Convex, PrimComp->GetComponentTransform());
		TArray<FVector2f> EdgeRayDistances;
		RaycastAlongIntactShapeEdges(IntactShape, OtherPrimComps, EdgeRayDistances);

		for (int32 EdgeIndex = 0; EdgeIndex < IntactShape.Edges.Num(); ++EdgeIndex)
		{
			const FIntVector2& IntactEdge = IntactShape.Edges[EdgeIndex];
			FVector2f& EdgeRayDistance = EdgeRayDistances[EdgeIndex];
			const FQuat& EdgeRotation = IntactShape.EdgeRotations[EdgeIndex];
			if (EdgeRayDistance.X == FLT_MAX) // No ray intersection, keep edge intact
			{
				ensure(EdgeRayDistance.Y == FLT_MAX);
				const int32 VertIndexA = FindOrAddVertex(IntactShape.Vertices[IntactEdge.X], Vertices);
				const int32 VertIndexB = FindOrAddVertex(IntactShape.Vertices[IntactEdge.Y], Vertices);
				Edges.Add(FIntVector2(VertIndexA, VertIndexB));
				EdgeRotations.Add(EdgeRotation);
				continue;
			}
			const FVector& IntactVertA = IntactShape.Vertices[IntactEdge.X];
			const FVector& IntactVertB = IntactShape.Vertices[IntactEdge.Y];
			const FVector DirectionAB = (IntactVertB - IntactVertA).GetSafeNormal();

			if (EdgeRayDistance.X > 0.f) // If vert A is not inside other shape.
			{
				const FVector NewVertB = IntactVertA + DirectionAB * (EdgeRayDistance.X);
				const int32 VertIndexA = FindOrAddVertex(IntactVertA, Vertices);
				const int32 VertIndexB = FindOrAddVertex(NewVertB, Vertices);
				Edges.Add(FIntVector2(VertIndexA, VertIndexB));
				EdgeRotations.Add(EdgeRotation);
			}

			if (EdgeRayDistance.Y > 0.f) // If vert B is not inside other shape.
			{
				const FVector NewVertA = IntactVertB - DirectionAB * (EdgeRayDistance.Y);
				const int32 VertIndexA = FindOrAddVertex(NewVertA, Vertices);
				const int32 VertIndexB = FindOrAddVertex(IntactVertB, Vertices);
				Edges.Add(FIntVector2(VertIndexA, VertIndexB));
				EdgeRotations.Add(EdgeRotation);
			}
		}
		PopulateVertToEdges();

		IsCornerVertexList.Init(false, Vertices.Num());
		for (int32 VertexIndex = 0; VertexIndex < Vertices.Num(); ++VertexIndex)
		{
			IsCornerVertexList[VertexIndex] = VertToEdges[VertexIndex].Num() < 2;
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

	void FRopeCollisionShape::RaycastAlongIntactShapeEdges(
		const FRopeCollisionShape& IntactShape
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, TArray<FVector2f>& OutEdgeRayDistances
	) const
	{
		// FLT_MAX represents no ray-intersections
		OutEdgeRayDistances.Init(FVector2f(FLT_MAX, FLT_MAX), IntactShape.Edges.Num());

		FHitResult LatestHitResult;
		FCollisionQueryParams TraceParams = FCollisionQueryParams();
		for (int32 EdgeIndex = 0; EdgeIndex < IntactShape.Edges.Num(); ++EdgeIndex)
		{
			const FIntVector2& Edge = IntactShape.Edges[EdgeIndex];
			FVector2f& EdgeRayDistance = OutEdgeRayDistances[EdgeIndex];
			const FVector& VertA = IntactShape.Vertices[Edge.X];
			const FVector& VertB = IntactShape.Vertices[Edge.Y];
			for (UPrimitiveComponent* OtherPrimComp : OtherPrimComps)
			{
				if (!OtherPrimComp->LineTraceComponent(LatestHitResult, VertA, VertB, TraceParams))
				{
					continue;
				}
				if (LatestHitResult.bStartPenetrating || LatestHitResult.Distance < SMALL_NUMBER)
				{
					EdgeRayDistance.X = -1.f;
				}
				else if (LatestHitResult.Distance < EdgeRayDistance.X)
				{
					EdgeRayDistance.X = LatestHitResult.Distance;
				}
				if (!OtherPrimComp->LineTraceComponent(LatestHitResult, VertB, VertA, TraceParams))
				{
					continue;
				}
				if (LatestHitResult.bStartPenetrating || LatestHitResult.Distance < SMALL_NUMBER)
				{
					EdgeRayDistance.Y = -1.f;
				}
				else if (LatestHitResult.Distance < EdgeRayDistance.Y)
				{
					EdgeRayDistance.Y = LatestHitResult.Distance;
				}
			}
		}
	};

	int32 FRopeCollisionShape::FindOrAddVertex(const FVector& NewVert, TArray<FVector>& InOutVerts) const
	{
		auto DistancePredicate = [&](const FVector& Target)
			{
				return [&](const FVector& V)
					{
						return FVector::DistSquared(V, Target) <= TAUT_ROPE_SHAPE_MERGE_VERTEX_THRESHOLD_SQUARED;
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
		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			const FIntVector2& Edge = Edges[EdgeIndex];
			const FVector EdgeVertA = Vertices[Edge.X];
			const FVector EdgeVertB = Vertices[Edge.Y];
			DrawDebugLine(
				World
				, EdgeVertA
				, EdgeVertB
				, FColor::Blue
			);

			const FVector Center = (EdgeVertA + EdgeVertB) * 0.5f;
			DrawDebugLine(
				World
				, Center
				, Center + EdgeRotations[EdgeIndex].GetUpVector() * 5.f
				, FColor::Yellow
			);
		}
		for (int32 VertIndex = 0; VertIndex < Vertices.Num(); ++VertIndex)
		{
			const FVector& Vert = Vertices[VertIndex];
			DrawDebugSphere(
				World
				, Vert
				, 1.f
				, 4
				, FColor::Red
			);
			for (int32 EdgeIndex : VertToEdges[VertIndex])
			{
				DrawDebugLine(
					World
					, Vert
					, Vert + EdgeRotations[EdgeIndex].GetUpVector() * 5.f
					, FColor::Yellow
				);
			}
		}
	};
#endif // TAUT_ROPE_DEBUG_DRAWING
}