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
		, int32& InOutNextPointId
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
					// A new contact feature, so a new identity rather than the
					// replaced point's.
					Point Replacement(Hit);
					Replacement.Id = InOutNextPointId++;
					RopePoints[RemovePointIndex] = Replacement;
				}
				else
				{
					Point Inserted(Hit);
					Inserted.Id = InOutNextPointId++;
					RopePoints.insert(RopePoints.begin() + RemovePointIndex, Inserted);
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
		OutHitData.TiedHits.clear();
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
		OutHitData.TiedHits.clear();
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

			if (!bIsIntersection)
			{
				continue;
			}

			// A strict < here used to decide the whole thing, which meant an edge
			// reached at the same moment as the incumbent was discarded without ever
			// being reported. That happens whenever the sweep crosses a vertex: both
			// edges meeting there are crossed at the same sweep ratio, and the lower
			// edge index won purely by iteration order. On the wrap recordings that
			// coin flip decided whether the rope attached to a real silhouette edge
			// or to the triangulation diagonal lying flat in a face.
			if (SweepRatio < OutHitData.SweepRatio)
			{
				// The winner is chosen exactly as it always was, so which edge the
				// rope attaches to does not change. What changes is that the edge
				// being displaced is kept when it was tied, instead of vanishing.
				if (OutHitData.bIsHit
					&& Math::Abs(SweepRatio - OutHitData.SweepRatio) <= SweepRatioTieTolerance)
				{
					TiedHit Displaced;
					Displaced.Location = OutHitData.Location;
					Displaced.OnSweepEdgeLocation = OutHitData.OnSweepEdgeLocation;
					Displaced.ShapeIndex = OutHitData.ShapeIndex;
					Displaced.EdgeIndex = OutHitData.EdgeIndex;
					Displaced.SweepRatio = OutHitData.SweepRatio;
					OutHitData.TiedHits.push_back(Displaced);
				}
				else
				{
					// A clearly nearer hit, so nothing that tied with the old winner
					// ties with this one.
					OutHitData.TiedHits.clear();
				}

				OutHitData.bIsHit = true;
				OutHitData.Location = ClosestPointOnLine;
				OutHitData.OnSweepEdgeLocation = OnSweepEdgeLocation;
				OutHitData.SweepRatio = SweepRatio;
				OutHitData.bIsHitOnFirstTriangleSweep = bIsFirstTriangleSweep;
				OutHitData.RopePointIndex = RopePointIndex;
				OutHitData.EdgeIndex = EdgeIndex;
				OutHitData.ShapeIndex = ShapeIndex;
			}
			else if (OutHitData.bIsHit
				&& Math::Abs(SweepRatio - OutHitData.SweepRatio) <= SweepRatioTieTolerance)
			{
				TiedHit Tied;
				Tied.Location = ClosestPointOnLine;
				Tied.OnSweepEdgeLocation = OnSweepEdgeLocation;
				Tied.ShapeIndex = ShapeIndex;
				Tied.EdgeIndex = EdgeIndex;
				Tied.SweepRatio = SweepRatio;
				OutHitData.TiedHits.push_back(Tied);
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

	namespace
	{
		// Where along the rope's swept path a contact at InLocation happens, by
		// projecting SupportCorner through it onto the From->To line. Factored out
		// of GetTriangleLineIntersection unchanged so both the general and the
		// coplanar path produce sweep ratios on the same scale.
		void ComputeSweepPosition(
			const Vec3& FromCorner
			, const Vec3& ToCorner
			, const Vec3& SupportCorner
			, const Vec3& InLocation
			, Vec3& OutOnSweepEdgeLocation
			, float& OutSweepRatio
		)
		{
			const Vec3 RayOrigin = SupportCorner;
			const Vec3 RayDir = (InLocation - SupportCorner).GetSafeNormal();

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

		// Det is a scalar triple product, so it scales with the product of the
		// three lengths. Comparing it against an absolute epsilon made the test
		// meaningless at world scale: with edges of a few hundred units a
		// well-conditioned Det is ~5e6, so a fixed 1e-4 threshold amounts to a
		// relative tolerance of 1e-11, finer than the cancellation in the dot
		// product can resolve. Normalise it instead.
		const double DetScale = Edge1.Size() * Dir.Size() * Edge2.Size();
		const bool bIsCoplanar = DetScale <= 0.0
			|| Math::Abs(static_cast<double>(Det)) < static_cast<double>(KindaSmallNumber) * DetScale;

		if (bIsCoplanar)
		{
			// The sweep triangle and the edge lie in the same plane, where
			// Moller-Trumbore has no answer to give: the intersection is a segment,
			// not a point, and the method divides by Det.
			//
			// This is why the rope can slide flat across an edge without ever
			// registering it, and why that edge then shows up as a fresh collision
			// later, once the geometry tilts out of plane -- by which time the rope
			// line is already cutting through the solid.
			//
			// Reporting the in-plane overlap here does NOT fix that. Tried: clip the
			// edge against the triangle in-plane and return the end the rope reaches
			// first. The insertion does not remove the condition, because a point
			// placed on the edge is still coplanar, so the next collision iteration
			// finds the same overlap again. CollisionPhase inserted a point on all
			// 100 of its iterations in a single frame (3 points -> 102), and the
			// following frame hung. A flat surface has nothing to wrap, so the fix
			// belongs upstream: do not offer coplanar edges to the rope at all.
			//
			// Tightening this threshold does not help, and that is measured, not
			// assumed. Narrowing the reject band from KindaSmallNumber to
			// SmallNumber admitted 186 extra sweeps out of 8.2 million and produced
			// a bit-identical replay of a 1190 frame recording. The reason is in
			// `--analyse conditioning`: of 3,652,016 near-coplanar tests, 3,649,210
			// have Det *bitwise* zero. The boxes are axis aligned and the rope
			// slides within a face plane, so the triple product cancels exactly.
			// These are not badly conditioned, they are unsolvable -- InvDet below
			// would divide by zero -- so no epsilon, padding or reformulation of
			// this test reaches them.
			return false;
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
		ComputeSweepPosition(FromCorner, ToCorner, SupportCorner, OutLocation, OutOnSweepEdgeLocation, OutSweepRatio);

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
