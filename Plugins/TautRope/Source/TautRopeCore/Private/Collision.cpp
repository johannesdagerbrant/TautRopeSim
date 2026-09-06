#include "TautRopeCore/Collision.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/DebugDraw.h"
#include "TautRopeCore/Point.h"

namespace TautRope
{
	void SweepRemovePoint(
		std::vector<Point>& RopePoints
		, const int32 RemovePointIndex
		, const std::vector<CollisionShape>& Shapes
		, IDebugDraw* Debug
	)
	{
		TAUTROPE_ENSURE(RemovePointIndex > 0 && RemovePointIndex < Num(RopePoints));

		const bool bIsDebugDrawingActive = Debug != nullptr && Debug->WantsRemoveSweep();

		bool bFoundIntersections = false;
		Vec3 FromLocation = RopePoints[RemovePointIndex].Location;
		Vec3 ToLocation = RopePoints[RemovePointIndex - 1].Location;
		Vec3 SupportLocation = RopePoints[RemovePointIndex + 1].Location;

		std::vector<Int2> IgnoredEdges = {
			Int2(RopePoints[RemovePointIndex - 1].ShapeIndex, RopePoints[RemovePointIndex - 1].EdgeIndex)
			, Int2(RopePoints[RemovePointIndex].ShapeIndex, RopePoints[RemovePointIndex].EdgeIndex)
			, Int2(RopePoints[RemovePointIndex + 1].ShapeIndex, RopePoints[RemovePointIndex + 1].EdgeIndex)
		};

		HitData Hit;
		Hit.bIsHit = true;
		while (Hit.bIsHit)
		{
			Hit = HitData();
			for (int32 ShapeIndex = 0; ShapeIndex < Num(Shapes); ++ShapeIndex)
			{
				SweepRemoveTriangleAgainstShape(
					FromLocation
					, ToLocation
					, SupportLocation
					, Shapes[ShapeIndex]
					, ShapeIndex
					, IgnoredEdges
					, Hit
				);
			}
			if (bIsDebugDrawingActive)
			{
				if (Hit.bIsHit)
				{
					DebugDrawSweep(*Debug, FromLocation, Hit.OnSweepEdgeLocation, SupportLocation, true);
				}
				else
				{
					DebugDrawSweep(*Debug, FromLocation, ToLocation, SupportLocation, false);
				}
			}
			if (Hit.bIsHit)
			{
				if (!bFoundIntersections)
				{
					bFoundIntersections = true;
					RopePoints[RemovePointIndex] = Point(Hit);
				}
				else
				{
					RopePoints.insert(RopePoints.begin() + RemovePointIndex, Point(Hit));
				}
				FromLocation = Hit.OnSweepEdgeLocation;
				ToLocation = RopePoints[RemovePointIndex - 1].Location;
				SupportLocation = RopePoints[RemovePointIndex].Location;
				IgnoredEdges = {
					Int2(RopePoints[RemovePointIndex - 1].ShapeIndex, RopePoints[RemovePointIndex - 1].EdgeIndex)
					, Int2(RopePoints[RemovePointIndex].ShapeIndex, RopePoints[RemovePointIndex].EdgeIndex)
				};
			}
		}
		if (!bFoundIntersections)
		{
			RopePoints.erase(RopePoints.begin() + RemovePointIndex);
		}
	}

	void SweepSegmentThroughShapes(
		HitData& OutHitData
		, Point& InOutSegmentPointA
		, Point& InOutSegmentPointB
		, const Vec3& OriginLocationA
		, const Vec3& OriginLocationB
		, const Vec3& TargetLocationA
		, const Vec3& TargetLocationB
		, const std::vector<CollisionShape>& Shapes
		, const int32 RopePointIndex
		, IDebugDraw* Debug
	)
	{
		const bool bIsDebugDrawingActive = Debug != nullptr && Debug->WantsSegmentSweep();

		OutHitData.SweepRatio = MaxFloat;
		// First perform triangle sweep for A-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Num(Shapes); ++ShapeIndex)
		{
			SweepSegmentTriangleAgainstShape(
				OriginLocationA,	// TriA
				TargetLocationA,	// TriB
				OriginLocationB,	// TriC
				Shapes[ShapeIndex],
				ShapeIndex,
				InOutSegmentPointA.ShapeIndex,	// ShapeIndexPointA
				InOutSegmentPointB.ShapeIndex,	// ShapeIndexPointB
				InOutSegmentPointA.EdgeIndex,	// EdgeIndexPointA
				InOutSegmentPointB.EdgeIndex,	// EdgeIndexPointB
				InOutSegmentPointA.VertIndex,	// VertIndexPointA
				InOutSegmentPointB.VertIndex,	// VertIndexPointB
				RopePointIndex,
				true,	// bIsFirstTriangleSweep
				OutHitData
			);
		}
		if (bIsDebugDrawingActive)
		{
			if (OutHitData.bIsHit)
			{
				DebugDrawSweep(*Debug, OriginLocationA, OutHitData.OnSweepEdgeLocation, OriginLocationB, true);
			}
			else
			{
				DebugDrawSweep(*Debug, OriginLocationA, TargetLocationA, OriginLocationB, false);
			}
		}
		if (OutHitData.bIsHit)
		{
			InOutSegmentPointA.Location = OutHitData.OnSweepEdgeLocation;
			return;
		}

		// No new collisions from A-movement triangle sweep
		InOutSegmentPointA.Location = TargetLocationA;

		OutHitData.SweepRatio = MaxFloat;
		// Perform triangle sweep for B-movement
		for (int32 ShapeIndex = 0; ShapeIndex < Num(Shapes); ++ShapeIndex)
		{
			const CollisionShape& Shape = Shapes[ShapeIndex];
			SweepSegmentTriangleAgainstShape(
				OriginLocationB,	// TriA
				TargetLocationB,	// TriB
				TargetLocationA,	// TriC
				Shape,
				ShapeIndex,
				InOutSegmentPointA.ShapeIndex,	// ShapeIndexPointA
				InOutSegmentPointB.ShapeIndex,	// ShapeIndexPointB
				InOutSegmentPointA.EdgeIndex,	// EdgeIndexPointA
				InOutSegmentPointB.EdgeIndex,	// EdgeIndexPointB
				InOutSegmentPointA.VertIndex,	// VertIndexPointA
				InOutSegmentPointB.VertIndex,	// VertIndexPointB
				RopePointIndex,
				false,	// bIsFirstTriangleSweep
				OutHitData
			);
		}
		if (bIsDebugDrawingActive)
		{
			if (OutHitData.bIsHit)
			{
				DebugDrawSweep(*Debug, OriginLocationB, OutHitData.OnSweepEdgeLocation, TargetLocationA, true);
			}
			else
			{
				DebugDrawSweep(*Debug, OriginLocationB, TargetLocationB, TargetLocationA, false);
			}
		}
		if (OutHitData.bIsHit)
		{
			InOutSegmentPointB.Location = OutHitData.OnSweepEdgeLocation;
			return;
		}

		// No new collisions from B-movement triangle sweep
		InOutSegmentPointB.Location = TargetLocationB;
	}

	void SweepSegmentTriangleAgainstShape(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const CollisionShape& Shape
		, const int32 ShapeIndex
		, const int32 ShapeIndexPointA
		, const int32 ShapeIndexPointB
		, const int32 EdgeIndexPointA
		, const int32 EdgeIndexPointB
		, const int32 VertIndexPointA
		, const int32 VertIndexPointB
		, const int32 RopePointIndex
		, const bool bIsFirstTriangleSweep
		, HitData& OutHitData
	)
	{
		for (int32 EdgeIndex = 0; EdgeIndex < Num(Shape.Edges); ++EdgeIndex)
		{
			if (ShapeIndex == ShapeIndexPointA)
			{
				if (EdgeIndex == EdgeIndexPointA)
					continue;

				if (VertIndexPointA != IndexNone && Contains(Shape.VertToEdges[VertIndexPointA], EdgeIndex))
					continue;
			}

			if (ShapeIndex == ShapeIndexPointB)
			{
				if (EdgeIndex == EdgeIndexPointB)
					continue;

				if (VertIndexPointB != IndexNone && Contains(Shape.VertToEdges[VertIndexPointB], EdgeIndex))
					continue;
			}

			const Int2& EdgeVerts = Shape.Edges[EdgeIndex];
			const Vec3& EdgeA = Shape.Vertices[EdgeVerts.X];
			const Vec3& EdgeB = Shape.Vertices[EdgeVerts.Y];

			Vec3 ClosestPointOnLine;
			Vec3 OnSweepEdgeLocation;
			float SweepRatio = MaxFloat;
			const bool bIsIntersection = GetTriangleLineIntersection(
				FromCorner
				, ToCorner
				, SupportCorner
				, EdgeA
				, EdgeB
				, ClosestPointOnLine
				, OnSweepEdgeLocation
				, SweepRatio
			);

			if (bIsIntersection && SweepRatio < OutHitData.SweepRatio)
			{
				OutHitData.bIsHit = true;
				OutHitData.Location = ClosestPointOnLine;
				OutHitData.OnSweepEdgeLocation = OnSweepEdgeLocation;
				OutHitData.SweepRatio = SweepRatio;
				OutHitData.bIsHitOnFirstTriangleSweep = bIsFirstTriangleSweep;
				OutHitData.RopePointIndex = RopePointIndex;
				OutHitData.EdgeIndex = EdgeIndex;
				OutHitData.ShapeIndex = ShapeIndex;
			}
		}
	}

	void SweepRemoveTriangleAgainstShape(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const CollisionShape& Shape
		, const int32 ShapeIndex
		, const std::vector<Int2>& IgnoredEdges
		, HitData& OutHitData
	)
	{
		for (int32 EdgeIndex = 0; EdgeIndex < Num(Shape.Edges); ++EdgeIndex)
		{
			if (Contains(IgnoredEdges, Int2(ShapeIndex, EdgeIndex)))
			{
				continue;
			}
			const Int2& EdgeVerts = Shape.Edges[EdgeIndex];
			const Vec3& EdgeA = Shape.Vertices[EdgeVerts.X];
			const Vec3& EdgeB = Shape.Vertices[EdgeVerts.Y];

			Vec3 ClosestPointOnLine;
			Vec3 OnSweepEdgeLocation;
			float SweepRatio = MaxFloat;
			const bool bIsIntersection = GetTriangleLineIntersection(
				FromCorner
				, ToCorner
				, SupportCorner
				, EdgeA
				, EdgeB
				, ClosestPointOnLine
				, OnSweepEdgeLocation
				, SweepRatio
			);
			if (bIsIntersection && SweepRatio < OutHitData.SweepRatio)
			{
				OutHitData.bIsHit = true;
				OutHitData.Location = ClosestPointOnLine;
				OutHitData.OnSweepEdgeLocation = OnSweepEdgeLocation;
				OutHitData.SweepRatio = SweepRatio;
				OutHitData.EdgeIndex = EdgeIndex;
				OutHitData.ShapeIndex = ShapeIndex;
			}
		}
	}

	bool GetTriangleLineIntersection(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const Vec3& LineA
		, const Vec3& LineB
		, Vec3& OutLocation
		, Vec3& OutOnSweepEdgeLocation
		, float& OutSweepRatio
	)
	{
		// --- Step 1: Moller-Trumbore triangle-line intersection ---
		const Vec3 Dir = LineB - LineA;
		const Vec3 Edge1 = ToCorner - FromCorner;
		const Vec3 Edge2 = SupportCorner - FromCorner;

		const Vec3 PVec = Vec3::Cross(Dir, Edge2);
		const float Det = static_cast<float>(Vec3::Dot(Edge1, PVec));
		if (Math::Abs(Det) < KindaSmallNumber)
		{
			return false; // Line parallel to triangle
		}

		const float InvDet = 1.0f / Det;
		const Vec3 TVec = LineA - FromCorner;

		const float U = static_cast<float>(Vec3::Dot(TVec, PVec)) * InvDet;
		if (U < 0.f || U > 1.0f)
		{
			return false;
		}

		const Vec3 QVec = Vec3::Cross(TVec, Edge1);
		const float V = static_cast<float>(Vec3::Dot(Dir, QVec)) * InvDet;
		if (V < 0.f || U + V > 1.0f)
		{
			return false;
		}

		const float T = static_cast<float>(Vec3::Dot(Edge2, QVec)) * InvDet;
		if (T < 0.f || T > 1.f)
		{
			return false;
		}

		OutLocation = LineA + Dir * T;

		// --- Step 2: Compute continuation along SupportCorner to OutLocation ---
		const Vec3 RayOrigin = SupportCorner;
		const Vec3 RayDir = (OutLocation - SupportCorner).GetSafeNormal();

		const Vec3 LinePoint = FromCorner;
		const Vec3 LineDir = ToCorner - FromCorner;

		const Vec3 CrossDir = Vec3::Cross(LineDir, RayDir);
		const float Denom = static_cast<float>(CrossDir.SizeSquared());

		if (Denom > KindaSmallNumber)
		{
			const float t = static_cast<float>(Vec3::Dot(Vec3::Cross(RayOrigin - LinePoint, RayDir), CrossDir)) / Denom;

			OutSweepRatio = t / static_cast<float>(LineDir.Size()); // normalized ratio along rope edge
			OutSweepRatio = Math::Clamp(OutSweepRatio, 0.f, 1.f);

			OutOnSweepEdgeLocation = LinePoint + LineDir * t;
		}
		else
		{
			// Ray and edge are parallel, fall back to nearest edge point
			OutSweepRatio = 0.f;
			OutOnSweepEdgeLocation = FromCorner;
		}

		return true;
	}

	void DebugDrawSweep(
		IDebugDraw& Debug
		, const Vec3& A
		, const Vec3& B
		, const Vec3& C
		, bool bIsHit
	)
	{
		(void)bIsHit;
		Debug.Triangle(A, B, C, ColorMagenta);
	}
}
