
#include "TautRopeHelpersCollision.h"
#include "TautRopeConfig.h"
#include "TautRopePoint.h"

namespace TautRope
{
	void SweepSegmentThroughShapes(
		FHitData& OutHitData
		, FPoint& InOutSegmentPointA
		, FPoint& InOutSegmentPointB
		, const FVector& OriginLocationA
		, const FVector& OriginLocationB
		, const FVector& TargetLocationA
		, const FVector& TargetLocationB
		, const TArray<FRopeCollisionShape>& Shapes
		, const int32 RopePointIndex
	)
	{
		float LowestBaryU = FLT_MAX;
		// First perform triangle sweep for A-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
		{
			// TODO: for performance, early-out when AABB checks fail between segment (A,B,TargetA,TargetB) and shape.

			const FRopeCollisionShape& Shape = Shapes[ShapeIndex];
			for (int32 EdgeIndex = 0; EdgeIndex < Shape.Edges.Num(); ++EdgeIndex)
			{
				if (ShapeIndex == InOutSegmentPointA.ShapeIndex)
				{
					if (EdgeIndex == InOutSegmentPointA.EdgeIndex)
					{
						continue;
					}
					if (InOutSegmentPointA.VertIndex != INDEX_NONE)
					{
						if (Shape.VertToEdges[InOutSegmentPointA.VertIndex].Contains(EdgeIndex))
						{
							continue;
						}
					}
				}
				if (ShapeIndex == InOutSegmentPointB.ShapeIndex)
				{
					if (EdgeIndex == InOutSegmentPointB.EdgeIndex)
					{
						continue;
					}
					if (InOutSegmentPointB.VertIndex != INDEX_NONE)
					{
						if (Shape.VertToEdges[InOutSegmentPointB.VertIndex].Contains(EdgeIndex))
						{
							continue;
						}
					}
				}
				const FIntVector2& EdgeVerts = Shape.Edges[EdgeIndex];
				const FVector& EdgeA = Shape.Vertices[EdgeVerts.X];
				const FVector& EdgeB = Shape.Vertices[EdgeVerts.Y];
				FHitData HitData = FHitData();
				const bool bIsIntersection = GetTriangleLineIntersection(OriginLocationA, TargetLocationA, OriginLocationB, EdgeA, EdgeB, HitData);
				if (!bIsIntersection)
				{
					continue;
				}
				if (HitData.BaryCoords.Y >= LowestBaryU)
				{
					continue;
				}
				LowestBaryU = HitData.BaryCoords.Y;
				HitData.EdgeIndex = EdgeIndex;
				HitData.bIsHit = true;
				HitData.bIsHitOnFirstTriangleSweep = true;
				HitData.ShapeIndex = ShapeIndex;
				HitData.RopePointIndex = RopePointIndex;
				OutHitData = HitData;
			}
		}
		if (OutHitData.bIsHit)
		{
			InOutSegmentPointA.Location = FMath::Lerp(OriginLocationA, TargetLocationA, LowestBaryU);
			return;
		}
		// No new collisions from A-movement triangle sweep: Set new A to equal target A
		InOutSegmentPointA.Location = TargetLocationA;

		// Perform triangle sweep for B-movement
		LowestBaryU = FLT_MAX;
		for (int32 ShapeIndex = 0; ShapeIndex < Shapes.Num(); ++ShapeIndex)
		{
			// TODO: for performance, early-out on cached bitarray of AABB checks from A-movement above.

			const FRopeCollisionShape& Shape = Shapes[ShapeIndex];
			for (int32 EdgeIndex = 0; EdgeIndex < Shape.Edges.Num(); ++EdgeIndex)
			{
				if (ShapeIndex == InOutSegmentPointA.ShapeIndex)
				{
					if (EdgeIndex == InOutSegmentPointA.EdgeIndex)
					{
						continue;
					}
					if (InOutSegmentPointA.VertIndex != INDEX_NONE)
					{
						if (Shape.VertToEdges[InOutSegmentPointA.VertIndex].Contains(EdgeIndex))
						{
							continue;
						}
					}
				}
				if (ShapeIndex == InOutSegmentPointB.ShapeIndex)
				{
					if (EdgeIndex == InOutSegmentPointB.EdgeIndex)
					{
						continue;
					}
					if (InOutSegmentPointB.VertIndex != INDEX_NONE)
					{
						if (Shape.VertToEdges[InOutSegmentPointB.VertIndex].Contains(EdgeIndex))
						{
							continue;
						}
					}
				}
				const FIntVector2& EdgeVerts = Shape.Edges[EdgeIndex];
				const FVector& EdgeA = Shape.Vertices[EdgeVerts.X];
				const FVector& EdgeB = Shape.Vertices[EdgeVerts.Y];
				FHitData HitData = FHitData();
				const bool bIsIntersection = GetTriangleLineIntersection(OriginLocationB, TargetLocationB, TargetLocationA, EdgeA, EdgeB, HitData);
				if (!bIsIntersection)
				{
					continue;
				}
				if (HitData.BaryCoords.Y >= LowestBaryU)
				{
					continue;
				}
				LowestBaryU = HitData.BaryCoords.Y;
				HitData.EdgeIndex = EdgeIndex;
				HitData.bIsHit = true;
				HitData.bIsHitOnFirstTriangleSweep = false;
				HitData.ShapeIndex = ShapeIndex;
				HitData.RopePointIndex = RopePointIndex;
				OutHitData = HitData;
			}
		}
		if (OutHitData.bIsHit)
		{
			InOutSegmentPointB.Location = FMath::Lerp(OriginLocationB, TargetLocationB, LowestBaryU);
			return;
		}
		// No new collisions from B-movement triangle sweep: Set new B to equal target B
		InOutSegmentPointB.Location = TargetLocationB;
	};

	bool GetTriangleLineIntersection(
		const FVector& TriA
		, const FVector& TriB
		, const FVector& TriC
		, const FVector& LineA
		, const FVector& LineB
		, FHitData& OutTautRopeHitData
	)
	{
		const FVector Dir = LineB - LineA;
		const FVector Edge1 = TriB - TriA;
		const FVector Edge2 = TriC - TriA;

		const FVector PVec = FVector::CrossProduct(Dir, Edge2);
		const float Det = FVector::DotProduct(Edge1, PVec);

		if (FMath::Abs(Det) < KINDA_SMALL_NUMBER)
		{
			return false;
		}

		const float InvDet = 1.0f / Det;
		const FVector TVec = LineA - TriA;
		const float U = FVector::DotProduct(TVec, PVec) * InvDet;
		if (U < 0.0f || U > 1.0f)
		{
			return false;
		}

		const FVector QVec = FVector::CrossProduct(TVec, Edge1);
		const float V = FVector::DotProduct(Dir, QVec) * InvDet;
		if (V < 0.0f || U + V > 1.0f)
		{
			return false;
		}
		const float T = FVector::DotProduct(Edge2, QVec) * InvDet;
		if (T >= 1.f || T <= 0.f)
		{
			return false;
		}
		OutTautRopeHitData.T = T;
		OutTautRopeHitData.Location = LineA + Dir * OutTautRopeHitData.T;
		OutTautRopeHitData.Distance = Dir.Size() * OutTautRopeHitData.T;
		const float W = 1.0f - U - V;
		OutTautRopeHitData.BaryCoords = FVector(W, U, V);

		return true;
	};
}