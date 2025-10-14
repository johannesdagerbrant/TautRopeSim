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

		for (int32 EdgeIndex = 0; EdgeIndex < IntactShape.Edges.Num(); ++EdgeIndex)
		{
			const FIntVector2& IntactEdge = IntactShape.Edges[EdgeIndex];
			const FQuat& EdgeRotation = IntactShape.EdgeRotations[EdgeIndex];
			const FVector& VertA = IntactShape.Vertices[IntactEdge.X];
			const FVector& VertB = IntactShape.Vertices[IntactEdge.Y];
			FHitResult InitHitResultA = FHitResult();
			FHitResult InitHitResultB = FHitResult();
			MakeInitialHitResults(InitHitResultA, InitHitResultB, VertA, VertB, OtherPrimComps);
			CreateIntermedateEdges(InitHitResultA, InitHitResultB, EdgeRotation, OtherPrimComps);
		}
		PopulateVertToEdges();

		IsCornerVertexList.Init(false, Vertices.Num());
		for (int32 VertexIndex = 0; VertexIndex < Vertices.Num(); ++VertexIndex)
		{
			IsCornerVertexList[VertexIndex] = VertToEdges[VertexIndex].Num() < 2;
		}
    };

	void FRopeCollisionShape::MakeInitialHitResults(
		FHitResult& InitHitResultA
		, FHitResult& InitHitResultB
		, const FVector& A
		, const FVector& B
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, const FCollisionQueryParams& TraceParams
	) const
	{
		const FVector AB = (B - A).GetSafeNormal() * TAUT_ROPE_DISTANCE_TOLERANCE;
		bool bIsHitA = false;
		for (UPrimitiveComponent* OtherPrimComp : OtherPrimComps)
		{
			if (OtherPrimComp->LineTraceComponent(InitHitResultA, A - AB, A + AB, TraceParams))
			{
				bIsHitA = true;
				break;
			}
		}
		if (bIsHitA)
		{
			InitHitResultA.bStartPenetrating = true;
			InitHitResultA.Distance = -1.f;
		}
		else
		{
			InitHitResultA.bStartPenetrating = false;
			InitHitResultA.Distance = FLT_MAX;
		}
		InitHitResultA.Location = A;

		bool bIsHitB = false;
		for (UPrimitiveComponent* OtherPrimComp : OtherPrimComps)
		{
			if (OtherPrimComp->LineTraceComponent(InitHitResultB, B + AB, B - AB, TraceParams))
			{
				bIsHitB = true;
				break;
			}
		}
		if (bIsHitB)
		{
			InitHitResultB.bStartPenetrating = true;
			InitHitResultB.Distance = -1.f;
		}
		else
		{
			InitHitResultB.bStartPenetrating = false;
			InitHitResultB.Distance = FLT_MAX;
		}
		InitHitResultB.Location = B;
	}

	void FRopeCollisionShape::CreateIntermedateEdges(
		const FHitResult& LastHitResultA
		, const FHitResult& LastHitResultB
		, const FQuat& EdgeRotation
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, const FCollisionQueryParams& TraceParams
	)
	{
		if (
			LastHitResultA.Distance < FLT_MAX 
			&& LastHitResultB.Distance < FLT_MAX
			&& LastHitResultA.Component == LastHitResultB.Component
			&& LastHitResultA.ElementIndex == LastHitResultB.ElementIndex
		)
		{
			return;
		}
		const FVector& A = LastHitResultA.Location;
		const FVector& B = LastHitResultB.Location;
		if (FVector::DistSquared(A, B) < TAUT_ROPE_SHAPE_EDGE_RAY_INCREMENT_DISTANCE_SQUARED)
		{
			return;
		}
		const FVector Middle = (A + B) * 0.5f;
		FHitResult NewHitResultA = FHitResult();
		FHitResult NewHitResultB = FHitResult();
		NewHitResultA.Distance = FLT_MAX;
		NewHitResultB.Distance = FLT_MAX;

		for (UPrimitiveComponent* OtherPrimComp : OtherPrimComps)
		{
			FHitResult ItrHitResultA;
			FHitResult ItrHitResultB;
			const bool bHitA = OtherPrimComp->LineTraceComponent(ItrHitResultA, Middle, A, TraceParams);
			const bool bHitB = OtherPrimComp->LineTraceComponent(ItrHitResultB, Middle, B, TraceParams);
			if (!bHitA && !bHitB)
			{
				continue;
			}

			if (bHitA && (ItrHitResultA.bStartPenetrating || ItrHitResultA.Distance < SMALL_NUMBER))
			{
				// if one ray starts inside other shape, the other should too, since they both originate from Middle.
				ensure(bHitB && (ItrHitResultB.bStartPenetrating || ItrHitResultB.Distance < SMALL_NUMBER));
				NewHitResultA = ItrHitResultA;
				NewHitResultB = ItrHitResultB;
				NewHitResultA.Distance = -1.f;
				NewHitResultB.Distance = -1.f;
				break;
			}
			if (bHitA && ItrHitResultA.Distance < NewHitResultA.Distance)
			{
				NewHitResultA = ItrHitResultA;
			}
			if (bHitB && ItrHitResultB.Distance < NewHitResultB.Distance)
			{
				NewHitResultB = ItrHitResultB;
			}
		}

		NewHitResultA.Location = Middle;
		NewHitResultB.Location = Middle;
		if (NewHitResultA.Distance > 0.f && NewHitResultB.Distance > 0.f)
		{
			if (NewHitResultA.Distance == FLT_MAX)
			{
				NewHitResultA.Location = A;
			}
			else
			{
				NewHitResultA.Location = (Middle + (A - Middle).GetSafeNormal() * (NewHitResultA.Distance + TAUT_ROPE_DISTANCE_TOLERANCE));
			}

			if (NewHitResultB.Distance == FLT_MAX)
			{
				NewHitResultB.Location = B;
			}
			else
			{
				NewHitResultB.Location = (Middle + (B - Middle).GetSafeNormal() * (NewHitResultB.Distance + TAUT_ROPE_DISTANCE_TOLERANCE));
			}
			const int32 VertIndexA = FindOrAddVertex(NewHitResultA.Location, Vertices);
			const int32 VertIndexB = FindOrAddVertex(NewHitResultB.Location, Vertices);
			Edges.Add(FIntVector2(VertIndexA, VertIndexB));
			EdgeRotations.Add(EdgeRotation);
		}
		CreateIntermedateEdges(LastHitResultA, NewHitResultA, EdgeRotation, OtherPrimComps, TraceParams);
		CreateIntermedateEdges(NewHitResultB, LastHitResultB, EdgeRotation, OtherPrimComps, TraceParams);
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
				, Center + EdgeRotations[EdgeIndex].GetUpVector() * FVector::Dist(EdgeVertA, EdgeVertB) * 0.05f
				, FColor::Yellow
			);
		}
		for (int32 VertIndex = 0; VertIndex < Vertices.Num(); ++VertIndex)
		{
			DrawDebugSphere(
				World
				, Vertices[VertIndex]
				, 0.5f
				, 4
				, IsCornerVertex(VertIndex) ? FColor::Red : FColor::Yellow
			);
		}
	};
#endif // TAUT_ROPE_DEBUG_DRAWING
}