#include "TautRopeCollisionShape.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"

namespace TautRope
{
	FRopeCollisionShape::FRopeCollisionShape(const FKConvexElem& Convex, const FTransform& CompTransform)
	{
		TArray<FIntVector> Triangles;
		for (int32 TriIndex = 0; TriIndex < Convex.IndexData.Num(); TriIndex += 3)
		{
			const int32 TriVertIndexA = Convex.IndexData[TriIndex];
			const int32 TriVertIndexB = Convex.IndexData[TriIndex + 1];
			const int32 TriVertIndexC = Convex.IndexData[TriIndex + 2];

			int32 ShapeVertIndexA = FindOrAddVertex(Convex.VertexData[TriVertIndexA]);
			int32 ShapeVertIndexB = FindOrAddVertex(Convex.VertexData[TriVertIndexB]);
			int32 ShapeVertIndexC = FindOrAddVertex(Convex.VertexData[TriVertIndexC]);
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

		TArray<FIntVector> TriIndexToEdgeIndex;
		TMap<int32, TArray<FIntVector>> EdgeIndexToTriangles;
		for (const FIntVector& Tri : Triangles)
		{
			const int32 EdgeIndexA = AddUniqueEdge(Tri.X, Tri.Y);
			const int32 EdgeIndexB = AddUniqueEdge(Tri.Y, Tri.Z);
			const int32 EdgeIndexC = AddUniqueEdge(Tri.Z, Tri.X);
			TriIndexToEdgeIndex.Add(FIntVector(EdgeIndexA, EdgeIndexB, EdgeIndexC));

			EdgeIndexToTriangles.FindOrAdd(EdgeIndexA).Add(Tri);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexB).Add(Tri);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexC).Add(Tri);
		}
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
	}

	int32 FRopeCollisionShape::FindOrAddVertex(FVector Vertex)
	{
		auto DistancePredicate = [&](const FVector& Target)
			{
				return [&](const FVector& V)
					{
						return FVector::DistSquared(V, Target) <= TAUT_ROPE_DISTANCE_TOLERANCE_SQUARED;
					};
			};
		int32 VertIndex = Vertices.IndexOfByPredicate(DistancePredicate(Vertex));
		if (VertIndex == INDEX_NONE)
		{
			return Vertices.Add(Vertex);
		}
		return VertIndex;
	};
	int32 FRopeCollisionShape::AddUniqueEdge(int32 V1, int32 V2)
	{
		return Edges.AddUnique((V1 < V2) ? FIntVector2(V1, V2) : FIntVector2(V2, V1));
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
	}

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
	}
#endif // TAUT_ROPE_DEBUG_DRAWING
}