#include "TautRopeShapeBuilder.h"

#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"
#include "TautRopeConfig.h"
#include "TautRopeConvert.h"

using TautRope::CollisionShape;
using TautRope::Int2;
using TautRope::Vec3;
using TautRopeConvert::ToCore;
using TautRopeConvert::ToUE;

namespace
{
	int32 FindOrAddVertex(const Vec3& NewVert, std::vector<Vec3>& InOutVerts)
	{
		for (int32 Index = 0; Index < static_cast<int32>(InOutVerts.size()); ++Index)
		{
			if ((InOutVerts[Index] - NewVert).SizeSquared() <= TAUT_ROPE_SHAPE_MERGE_VERTEX_THRESHOLD_SQUARED)
			{
				return Index;
			}
		}
		InOutVerts.push_back(NewVert);
		return static_cast<int32>(InOutVerts.size()) - 1;
	}

	int32 AddUniqueEdge(int32 V1, int32 V2, std::vector<Int2>& InOutEdges)
	{
		const Int2 Edge = (V1 < V2) ? Int2(V1, V2) : Int2(V2, V1);
		for (int32 Index = 0; Index < static_cast<int32>(InOutEdges.size()); ++Index)
		{
			if (InOutEdges[Index] == Edge)
			{
				return Index;
			}
		}
		InOutEdges.push_back(Edge);
		return static_cast<int32>(InOutEdges.size()) - 1;
	}

	int32 AddUniqueTriangle(int32 V1, int32 V2, int32 V3, TArray<FIntVector>& InOutTriangles)
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

	void PopulateVertToEdges(CollisionShape& Shape)
	{
		Shape.VertToEdges.clear();
		Shape.VertToEdges.resize(Shape.Vertices.size());
		for (int32 VertIndex = 0; VertIndex < static_cast<int32>(Shape.Vertices.size()); ++VertIndex)
		{
			for (int32 EdgeIndex = 0; EdgeIndex < static_cast<int32>(Shape.Edges.size()); ++EdgeIndex)
			{
				const Int2& Edge = Shape.Edges[EdgeIndex];
				if (VertIndex == Edge.X || VertIndex == Edge.Y)
				{
					Shape.VertToEdges[VertIndex].push_back(EdgeIndex);
				}
			}
		}
	}

	void MakeInitialHitResults(
		FHitResult& InitHitResultA
		, FHitResult& InitHitResultB
		, const FVector& A
		, const FVector& B
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, const FCollisionQueryParams& TraceParams = FCollisionQueryParams()
	)
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

	void CreateIntermedateEdges(
		CollisionShape& Shape
		, const FHitResult& LastHitResultA
		, const FHitResult& LastHitResultB
		, const FQuat& EdgeRotation
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
		, const FCollisionQueryParams& TraceParams = FCollisionQueryParams()
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
			const int32 VertIndexA = FindOrAddVertex(ToCore(NewHitResultA.Location), Shape.Vertices);
			const int32 VertIndexB = FindOrAddVertex(ToCore(NewHitResultB.Location), Shape.Vertices);
			Shape.Edges.push_back(Int2(VertIndexA, VertIndexB));
			Shape.EdgeRotations.push_back(ToCore(EdgeRotation));
		}
		CreateIntermedateEdges(Shape, LastHitResultA, NewHitResultA, EdgeRotation, OtherPrimComps, TraceParams);
		CreateIntermedateEdges(Shape, NewHitResultB, LastHitResultB, EdgeRotation, OtherPrimComps, TraceParams);
	}
}

namespace TautRopeShapeBuilder
{
	CollisionShape Build(const FKConvexElem& Convex, const FTransform& ComponentTransform)
	{
		CollisionShape Shape;

		TArray<FIntVector> Triangles;
		for (int32 TriIndex = 0; TriIndex < Convex.IndexData.Num(); TriIndex += 3)
		{
			const int32 TriVertIndexA = Convex.IndexData[TriIndex];
			const int32 TriVertIndexB = Convex.IndexData[TriIndex + 1];
			const int32 TriVertIndexC = Convex.IndexData[TriIndex + 2];

			int32 ShapeVertIndexA = FindOrAddVertex(ToCore(Convex.VertexData[TriVertIndexA]), Shape.Vertices);
			int32 ShapeVertIndexB = FindOrAddVertex(ToCore(Convex.VertexData[TriVertIndexB]), Shape.Vertices);
			int32 ShapeVertIndexC = FindOrAddVertex(ToCore(Convex.VertexData[TriVertIndexC]), Shape.Vertices);
			const FVector VertA = ToUE(Shape.Vertices[ShapeVertIndexA]);
			const FVector VertB = ToUE(Shape.Vertices[ShapeVertIndexB]);
			const FVector VertC = ToUE(Shape.Vertices[ShapeVertIndexC]);
			const FVector TriCross = FVector::CrossProduct(VertB - VertA, VertC - VertA);
			const float TriArea = TriCross.Size() * 0.5f;
			if (TriArea < KINDA_SMALL_NUMBER)
			{
				continue;
			}
			AddUniqueTriangle(ShapeVertIndexA, ShapeVertIndexB, ShapeVertIndexC, Triangles);
		}

		const FTransform ConvexTransform = Convex.GetTransform() * ComponentTransform;
		for (Vec3& Vert : Shape.Vertices)
		{
			Vert = ToCore(ConvexTransform.TransformPosition(ToUE(Vert)));
		}

		TMap<int32, TArray<FIntVector>> EdgeIndexToTriangles;
		for (const FIntVector& Tri : Triangles)
		{
			const int32 EdgeIndexA = AddUniqueEdge(Tri.X, Tri.Y, Shape.Edges);
			const int32 EdgeIndexB = AddUniqueEdge(Tri.Y, Tri.Z, Shape.Edges);
			const int32 EdgeIndexC = AddUniqueEdge(Tri.Z, Tri.X, Shape.Edges);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexA).Add(Tri);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexB).Add(Tri);
			EdgeIndexToTriangles.FindOrAdd(EdgeIndexC).Add(Tri);
		}

		PopulateVertToEdges(Shape);

		Shape.EdgeRotations.resize(Shape.Edges.size());
		for (auto [EdgeIndex, NeighborTriangles] : EdgeIndexToTriangles)
		{
			if (!ensure(!NeighborTriangles.IsEmpty()))
			{
				continue;
			}
			FVector TriangleNormalSum = FVector::ZeroVector;
			for (const FIntVector& NeighborTriangle : NeighborTriangles)
			{
				const FVector A = ToUE(Shape.Vertices[NeighborTriangle.X]);
				const FVector B = ToUE(Shape.Vertices[NeighborTriangle.Y]);
				const FVector C = ToUE(Shape.Vertices[NeighborTriangle.Z]);
				TriangleNormalSum += FVector::CrossProduct(B - A, C - A).GetSafeNormal();
			}
			const FVector Forward = (ToUE(Shape.Vertices[Shape.Edges[EdgeIndex].Y]) - ToUE(Shape.Vertices[Shape.Edges[EdgeIndex].X])).GetSafeNormal();
			const FVector Up = TriangleNormalSum.GetSafeNormal();
			Shape.EdgeRotations[EdgeIndex] = ToCore(FRotationMatrix::MakeFromXZ(Forward, Up).ToQuat());
		}

		Shape.IsCornerVertexList.assign(Shape.Vertices.size(), false);
		return Shape;
	}

	CollisionShape Build(
		const FKConvexElem& Convex
		, const UPrimitiveComponent* PrimComp
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
	)
	{
		const CollisionShape IntactShape = Build(Convex, PrimComp->GetComponentTransform());

		CollisionShape Shape;
		for (int32 EdgeIndex = 0; EdgeIndex < static_cast<int32>(IntactShape.Edges.size()); ++EdgeIndex)
		{
			const Int2& IntactEdge = IntactShape.Edges[EdgeIndex];
			const FQuat EdgeRotation(
				IntactShape.EdgeRotations[EdgeIndex].X
				, IntactShape.EdgeRotations[EdgeIndex].Y
				, IntactShape.EdgeRotations[EdgeIndex].Z
				, IntactShape.EdgeRotations[EdgeIndex].W
			);
			const FVector VertA = ToUE(IntactShape.Vertices[IntactEdge.X]);
			const FVector VertB = ToUE(IntactShape.Vertices[IntactEdge.Y]);
			FHitResult InitHitResultA = FHitResult();
			FHitResult InitHitResultB = FHitResult();
			MakeInitialHitResults(InitHitResultA, InitHitResultB, VertA, VertB, OtherPrimComps);
			CreateIntermedateEdges(Shape, InitHitResultA, InitHitResultB, EdgeRotation, OtherPrimComps);
		}
		PopulateVertToEdges(Shape);

		Shape.IsCornerVertexList.assign(Shape.Vertices.size(), false);
		for (int32 VertexIndex = 0; VertexIndex < static_cast<int32>(Shape.Vertices.size()); ++VertexIndex)
		{
			Shape.IsCornerVertexList[VertexIndex] = Shape.VertToEdges[VertexIndex].size() < 2;
		}
		return Shape;
	}
}

namespace
{
	// A box is a convex hull with eight corners, so it is expanded into one and
	// run through the same code rather than given a parallel implementation.
	// Winding is outward: Cross(B - A, C - A) points away from the box, which is
	// what the edge rotations derive their up vector from.
	FKConvexElem MakeConvexFromBox(const FKBoxElem& Box)
	{
		const double HalfX = Box.X * 0.5;
		const double HalfY = Box.Y * 0.5;
		const double HalfZ = Box.Z * 0.5;

		FKConvexElem Convex;
		Convex.VertexData = {
			FVector(-HalfX, -HalfY, -HalfZ),	// 0
			FVector(+HalfX, -HalfY, -HalfZ),	// 1
			FVector(+HalfX, +HalfY, -HalfZ),	// 2
			FVector(-HalfX, +HalfY, -HalfZ),	// 3
			FVector(-HalfX, -HalfY, +HalfZ),	// 4
			FVector(+HalfX, -HalfY, +HalfZ),	// 5
			FVector(+HalfX, +HalfY, +HalfZ),	// 6
			FVector(-HalfX, +HalfY, +HalfZ),	// 7
		};
		Convex.IndexData = {
			4, 5, 6,  4, 6, 7,	// +Z
			0, 3, 2,  0, 2, 1,	// -Z
			1, 2, 6,  1, 6, 5,	// +X
			0, 4, 7,  0, 7, 3,	// -X
			3, 7, 6,  3, 6, 2,	// +Y
			0, 1, 5,  0, 5, 4,	// -Y
		};
		Convex.SetTransform(FTransform(Box.Rotation, Box.Center));
		return Convex;
	}
}

namespace TautRopeShapeBuilder
{
	CollisionShape Build(
		const FKBoxElem& Box
		, const UPrimitiveComponent* PrimComp
		, const TArray<UPrimitiveComponent*>& OtherPrimComps
	)
	{
		return Build(MakeConvexFromBox(Box), PrimComp, OtherPrimComps);
	}
}
