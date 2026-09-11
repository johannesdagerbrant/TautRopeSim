// Minimal cases against Collision.h. Each builds a production CollisionShape by
// hand from the smallest data that shows the effect and calls the production
// sweep directly. No fixture, no Rope, no frame loop: if a defect needs any of
// those to appear, it is not understood well enough to be tested yet.

#include "Framework.h"

#include "TautRopeCore/Collision.h"
#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Point.h"
#include "TautRopeCore/Rope.h"

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

namespace
{
	TautRope::CollisionShape MakeSeamTestEdge(const Vec3& A, const Vec3& B)
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = { A, B };
		Shape.Edges = { TautRope::Int2(0, 1) };
		Shape.VertToEdges = { { 0 }, { 0 } };
		Shape.EdgeRotations = { TautRope::Quat() };
		Shape.IsCornerVertexList = { true, true };
		return Shape;
	}
}

// PROVES: removing a point that sits on one copy of a twin seam edge does not
// make the remove sweep re-find the other copy at the same spot - the ignored
// list is expanded with geometric twins, and the removal erases cleanly in one
// round without inserting anything.
// FIXES: the remove-sweep runaway on the lap recording (085126): IgnoredEdges
// matches exact shape/edge pairs, so the other hull's copy of the removed
// point's own segment was hit at the point's own location, and each round
// re-found the alternating copy - one zero-length insertion per iteration
// until the 100-round ceiling. Sabotage: drop AppendCoincidentTwinEdges and
// this goes red with the sweep at its ceiling and dozens of inserted points.
TEST(RemoveSweep_IgnoresGeometricTwinsOfIgnoredEdges)
{
	const TautRope::CollisionShape SeamA = MakeSeamTestEdge(Vec3(-50.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0));
	const TautRope::CollisionShape SeamB = MakeSeamTestEdge(Vec3(-50.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0));
	const std::vector<TautRope::CollisionShape> Shapes = { SeamA, SeamB };

	// One point on one copy of the seam, free anchors below it on either side:
	// the straightened rope no longer wraps the seam, so the removal should
	// erase the point. The sweep triangle's From corner lies ON the seam line,
	// which is exactly where the un-ignored twin copy gets hit without the
	// expansion.
	std::vector<TautRope::Point> Points(3);
	Points[0].Location = Vec3(-40.0, -40.0, -20.0);
	Points[1].Location = Vec3(-20.0, 0.0, 0.0);
	Points[1].ShapeIndex = 0;
	Points[1].EdgeIndex = 0;
	Points[1].Id = 1;
	Points[2].Location = Vec3(40.0, 40.0, -20.0);

	TautRope::int32 NextId = 2;
	TautRope::int32 Iterations = 0;
	TautRope::SweepRemovePoint(Points, 1, Shapes, NextId, nullptr, &Iterations);

	std::printf("      %d rounds, %d points after removal\n",
		Iterations, static_cast<int>(Points.size()));
	CHECK_EQ(Points.size(), std::size_t(2));
	CHECK(Iterations <= 1);
}

// PROVES: dragging a rope across a twin seam never duplicates points: each
// transit inserts at most one point per copy, because a hit landing on top of
// a segment endpoint is filtered as a zero-length duplicate and a filtered hit
// does not count as loop progress.
// FIXES: the collision-phase runaway on the lap recording (085126, frame
// 1203): after a tied crossing inserted one point per copy, the segment next
// to each twin kept re-finding the OTHER copy at the twin's own location - 51
// duplicates on one edge and 50 on its twin in one frame's hundred iterations.
// Sabotage, seen red both ways: disable the filter and the peak point count
// explodes; make progress hit-based again and the loop spins to its cap.
TEST(Collision_TwinSeamCrossingNeverDuplicatesPoints)
{
	TautRope::CollisionShape SeamA = MakeSeamTestEdge(Vec3(-50.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0));
	TautRope::CollisionShape SeamB = MakeSeamTestEdge(Vec3(-50.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0));

	TautRope::Rope Rope;
	Rope.AppendToNearbyShapes({ SeamA, SeamB });

	// One anchor stays below the seam; the other descends across it, so the
	// first contact already reads as wrapping and the twins latch instead of
	// being pruned mid-transit.
	const Vec3 AnchorA(-40.0, -25.0, -20.0);
	for (int Frame = 0; Frame <= 20; ++Frame)
	{
		const double H = 40.0 - 2.7 * Frame;
		Rope.UpdateRope(AnchorA, Vec3(40.0, 40.0, H), 1500.f);
	}

	std::printf("      peak %d points, worst %d collision iterations\n",
		Rope.MostRopePoints, Rope.MostCollisionIterations);
	CHECK(Rope.MostRopePoints <= 4);
	CHECK(Rope.MostCollisionIterations <= 2);
	CHECK_EQ(Rope.CollisionIterationCapHits, 0);
}
