// Minimal cases against Collision.h. Each builds a production CollisionShape by
// hand from the smallest data that shows the effect and calls the production
// sweep directly. No fixture, no Rope, no frame loop: if a defect needs any of
// those to appear, it is not understood well enough to be tested yet.

#include "Framework.h"

#include "TautRopeCore/Collision.h"
#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Point.h"

#include <cstdio>

using TautRope::Vec3;

namespace
{
	// Two edges meeting at one vertex. That is the whole geometry the tie needs:
	// a sweep triangle containing the shared vertex crosses both edges at exactly
	// that point, so both are reached at the same instant.
	TautRope::CollisionShape MakeTwoEdgesSharingAVertex()
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = { Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 1.0), Vec3(0.5, 0.5, 1.0) };
		Shape.Edges = { TautRope::Int2(0, 1), TautRope::Int2(0, 2) };
		Shape.VertToEdges = { { 0, 1 }, { 0 }, { 1 } };
		Shape.EdgeRotations = { TautRope::Quat(), TautRope::Quat() };
		Shape.IsCornerVertexList = { true, false, false };
		return Shape;
	}

	// In the z = 0 plane, containing the shared vertex at the origin.
	const Vec3 TriA(-1.0, -1.0, 0.0);
	const Vec3 TriB(1.0, -1.0, 0.0);
	const Vec3 TriC(0.0, 1.0, 0.0);

	TautRope::HitData SweepAgainst(const TautRope::CollisionShape& Shape)
	{
		TautRope::HitData Hit;
		TautRope::SweepSegmentTriangleAgainstShape(
			TriA, TriB, TriC
			, Shape
			, 0
			, TautRope::IndexNone, TautRope::IndexNone   // neither rope point is on a shape
			, TautRope::IndexNone, TautRope::IndexNone   // ...so no edge is skipped
			, TautRope::IndexNone, TautRope::IndexNone
			, 1
			, true
			, Hit
		);
		return Hit;
	}
}

// PROVES: when one sweep reaches two edges at the same instant, both are
// reported. The sweep triangle contains the vertex the two edges share, so each
// crosses it at exactly that point and neither is nearer than the other.
// FIXES: the rope silently skipping an edge. SweepSegmentTriangleAgainstShape
// kept a single HitData and compared with a strict <, so the second edge was
// discarded and the lower index won on iteration order alone. In recording
// 192009 that coin flip dropped the triangulation diagonal, and six frames after
// a converged group slid off vertex 1 the rope was 95.6 units inside the shape.
// Sabotage: drop the TiedHits branch from the sweep and this goes red.
TEST(Sweep_ReportsBothEdgesReachedAtTheSameInstant)
{
	const TautRope::CollisionShape Shape = MakeTwoEdgesSharingAVertex();
	const TautRope::HitData Hit = SweepAgainst(Shape);

	CHECK(Hit.bIsHit);
	CHECK_EQ(static_cast<int>(Hit.TiedHits.size()), 1);
	if (Hit.TiedHits.empty())
	{
		std::printf("      only edge %d was reported; the other was discarded\n", Hit.EdgeIndex);
		return;
	}

	// Both edges, in some order, and nothing else.
	const int First = Hit.EdgeIndex;
	const int Second = Hit.TiedHits[0].EdgeIndex;
	CHECK(First != Second);
	CHECK(First + Second == 1);

	// Same instant is the premise, so the ratios must be equal, not merely close.
	CHECK_BITS(Hit.TiedHits[0].SweepRatio, Hit.SweepRatio);
	std::printf("      edges %d and %d both reported at sweep ratio %.9g\n",
		First, Second, static_cast<double>(Hit.SweepRatio));
}

// PROVES: a sweep that crosses only one edge reports exactly one hit.
// GUARDS: the test above from passing for the wrong reason. If the sweep tied
// everything to everything, the tie assertion would hold while meaning nothing.
// Sabotage: report every intersecting edge as tied regardless of ratio and this
// goes red.
TEST(Sweep_ReportsNoTieWhenOnlyOneEdgeIsReached)
{
	// Two edges again, but crossing the triangle at clearly different points, so
	// they are reached at different instants. A tie here would be wrong.
	TautRope::CollisionShape Shape;
	// Offset in x, not y. Two crossings that differ only in y project through
	// SupportCorner onto the same point of the From-To line and tie after all,
	// which is worth knowing: the sweep ratio is a position along the rope, not a
	// distance from the triangle.
	Shape.Vertices = {
		Vec3(-0.3, -0.5, -1.0), Vec3(-0.3, -0.5, 1.0)   // edge 0 -> sweep ratio 0.3
		, Vec3(0.3, -0.5, -1.0), Vec3(0.3, -0.5, 1.0)   // edge 1 -> sweep ratio 0.7
	};
	Shape.Edges = { TautRope::Int2(0, 1), TautRope::Int2(2, 3) };
	Shape.VertToEdges = { { 0 }, { 0 }, { 1 }, { 1 } };
	Shape.EdgeRotations = { TautRope::Quat(), TautRope::Quat() };
	Shape.IsCornerVertexList = { false, false, false, false };

	const TautRope::HitData Hit = SweepAgainst(Shape);

	CHECK(Hit.bIsHit);
	CHECK_EQ(static_cast<int>(Hit.TiedHits.size()), 0);
	// The nearer edge wins outright.
	CHECK_EQ(Hit.EdgeIndex, 0);
}
