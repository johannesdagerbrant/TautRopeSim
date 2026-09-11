// Minimal cases against Pruning.h.

#include "Framework.h"

#include "TautRopeCore/Collision.h"
#include "TautRopeCore/Point.h"
#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Pruning.h"

#include <cstdio>
#include <vector>

using TautRope::Vec3;

namespace
{
	// Identity rotation puts the wrap plane's normal along +Y, so the test reduces
	// to the Y offsets of the two neighbours relative to the point being judged.
	const TautRope::Quat FlatEdge;

	const Vec3 Judged(0.0, 0.0, 0.0);

	// Same side in Y as Judged's other neighbour, so the pair reads as "not
	// wrapping". This is the point that is about to be removed anyway.
	const Vec3 DoomedNeighbour(-1.0, 1.0, 0.0);

	// The neighbour Judged actually ends up with once DoomedNeighbour is gone.
	const Vec3 SurvivingNeighbour(-2.0, -1.0, 0.0);

	const Vec3 OtherSide(1.0, 1.0, 0.0);
}

// PROVES: whether a point counts as wrapping depends on which neighbour it is
// measured against, and the answer flips when the neighbour changes.
// GUARDS: the single-pass removal decision in Rope::PruningPhase, which judges
// every point against RopePoints[i-1] and RopePoints[i+1] without checking
// whether those neighbours are themselves marked for removal in the same pass.
// When they are, the surviving rope is judged against a configuration that never
// exists.
TEST(Pruning_WrapVerdictDependsOnWhichNeighbourSurvives)
{
	const bool bAgainstDoomed = TautRope::IsRopeWrappingEdge(
		DoomedNeighbour, Judged, OtherSide, FlatEdge);
	const bool bAgainstSurvivor = TautRope::IsRopeWrappingEdge(
		SurvivingNeighbour, Judged, OtherSide, FlatEdge);

	std::printf("      wrapping against doomed neighbour   %s\n", bAgainstDoomed ? "yes" : "no");
	std::printf("      wrapping against surviving neighbour %s\n", bAgainstSurvivor ? "yes" : "no");

	// The point is dropped when judged against the neighbour that is leaving, and
	// kept when judged against the one that stays. Same point, same edge.
	CHECK(!bAgainstDoomed);
	CHECK(bAgainstSurvivor);
}

// PROVES: an edge is found or not found by the removal sweep purely according to
// whether it is in the ignored list, with the geometry held fixed.
// GUARDS: the ignored list in SweepRemovePoint, which is built with the edges of
// points [i-1], [i] and [i+1], and then REBUILT after the first hit with only
// [i-1] and [i]. From the second iteration onward the next rope point's edge is
// no longer protected, so the sweep may place a point on an edge its neighbour
// already occupies. The recordings reach three rounds, so the later iterations
// are not hypothetical.
TEST(RemoveSweep_NeighbourEdgeIsOnlyProtectedWhileItIsIgnored)
{
	TautRope::CollisionShape Shape;
	Shape.Vertices = { Vec3(0.0, -0.5, -1.0), Vec3(0.0, -0.5, 1.0) };
	Shape.Edges = { TautRope::Int2(0, 1) };
	Shape.VertToEdges = { { 0 }, { 0 } };
	Shape.EdgeRotations = { TautRope::Quat() };
	Shape.IsCornerVertexList = { false, false };

	const Vec3 TriA(-1.0, -1.0, 0.0);
	const Vec3 TriB(1.0, -1.0, 0.0);
	const Vec3 TriC(0.0, 1.0, 0.0);

	// First iteration: the neighbour's edge is in the list.
	TautRope::HitData Protected;
	const std::vector<TautRope::Int2> WithNeighbour = { TautRope::Int2(0, 0) };
	TautRope::SweepRemoveTriangleAgainstShape(TriA, TriB, TriC, Shape, 0, WithNeighbour, Protected);

	// Second iteration onward: the list has been rebuilt without it.
	TautRope::HitData Exposed;
	const std::vector<TautRope::Int2> WithoutNeighbour;
	TautRope::SweepRemoveTriangleAgainstShape(TriA, TriB, TriC, Shape, 0, WithoutNeighbour, Exposed);

	std::printf("      neighbour edge ignored -> hit %s\n", Protected.bIsHit ? "yes" : "no");
	std::printf("      neighbour edge exposed -> hit %s\n", Exposed.bIsHit ? "yes" : "no");

	CHECK(!Protected.bIsHit);
	CHECK(Exposed.bIsHit);
	CHECK_EQ(Exposed.EdgeIndex, 0);
}

// PROVES: a point whose wrap verdict would be judged against a neighbour that
// is already marked for removal in the same pass is kept this frame, not
// removed on the stale verdict.
// FIXES: the mass-arrival cascade on recording 154909. The fan movement solve
// makes whole vertex-cone groups arrive in one frame; the wrap verdict for the
// next point along, judged against a doomed cone member, said "not wrapping"
// while the survivor configuration wraps (the flip
// Pruning_WrapVerdictDependsOnWhichNeighbourSurvives isolates). Removing it put
// 92 units of rope inside a shape from frame 142 to the end of the recording;
// with the deferral the same replay has zero penetrating frames.
// Sabotage: drop the deferral from GetPointsToRemove and this goes red with the
// judged point marked.
TEST(Pruning_WrapVerdictIsDeferredWhileItsNeighbourIsDoomed)
{
	TautRope::CollisionShape Shape;
	Shape.Vertices = { Vec3(-2.0, 0.0, -1.0), Vec3(-2.0, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), Vec3(0.0, 0.0, 1.0) };
	Shape.Edges = { TautRope::Int2(0, 1), TautRope::Int2(2, 3) };
	Shape.VertToEdges = { { 0 }, { 0 }, { 1 }, { 1 } };
	Shape.EdgeRotations = { TautRope::Quat(), TautRope::Quat() };
	Shape.IsCornerVertexList = { true, true, true, true };

	// Two points on edge 0 make the second a same-edge duplicate, so it is
	// marked for removal without any geometry involved. The judged point sits
	// next on edge 1, placed so the wrap verdict against the doomed duplicate
	// reads "not wrapping" (both neighbours on the same side of its edge plane).
	std::vector<TautRope::Point> Points(5);
	Points[0].Location = Vec3(-3.0, -1.0, 0.0);

	Points[1].Location = Vec3(-2.0, 0.5, 0.0);
	Points[1].ShapeIndex = 0;
	Points[1].EdgeIndex = 0;

	Points[2].Location = DoomedNeighbour;
	Points[2].ShapeIndex = 0;
	Points[2].EdgeIndex = 0;

	Points[3].Location = Judged;
	Points[3].ShapeIndex = 0;
	Points[3].EdgeIndex = 1;

	Points[4].Location = OtherSide;

	const std::vector<bool> ToRemove = TautRope::GetPointsToRemove(Points, { Shape });

	// The duplicate is marked; the stale wrap verdict next to it is deferred.
	CHECK(ToRemove[2]);
	CHECK(!ToRemove[3]);

	const bool bAgainstDoomed = TautRope::IsRopeWrappingEdge(
		DoomedNeighbour, Judged, OtherSide, FlatEdge);
	CHECK(!bAgainstDoomed);
}

namespace
{
	// Two shapes each owning an edge along the same line - the seam - with the
	// twins a tied cross-shape sweep inserts stacked at one spot on it.
	TautRope::CollisionShape MakeSeamShape()
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = { Vec3(-50.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0) };
		Shape.Edges = { TautRope::Int2(0, 1) };
		Shape.VertToEdges = { { 0 }, { 0 } };
		Shape.EdgeRotations = { TautRope::Quat() };
		Shape.IsCornerVertexList = { true, true };
		return Shape;
	}
}

// PROVES: a cross-shape twin pair - two points at the same location on two
// shapes' seam edges - is marked for removal when the rope is straight, because
// each twin's wrap verdict is judged against the nearest neighbours at a
// DISTINCT location instead of against the twin on top of it.
// FIXES: the seam glue in recording 072336. The coincident twin gave
// IsRopeWrappingEdge a zero-length baseline, its degenerate branch answered
// "wrapping" forever, and 20 points stayed glued while the endpoint was dragged
// 28,853 units away. With distinct-neighbour verdicts the replay sheds to 2.
// Sabotage: judge against the immediate neighbours again and this goes red with
// both twins kept.
TEST(Pruning_CoincidentCrossShapeTwinsAreJudgedAgainstDistinctNeighbours)
{
	const TautRope::CollisionShape Seam = MakeSeamShape();

	// Both anchors on the same side of the seam edge's wrap plane: the rope is
	// straight past the seam and neither twin is wrapping anything.
	std::vector<TautRope::Point> Points(4);
	Points[0].Location = Vec3(-40.0, 40.0, 0.0);

	Points[1].Location = Vec3(-20.0, 0.0, 0.0);
	Points[1].ShapeIndex = 0;
	Points[1].EdgeIndex = 0;

	Points[2].Location = Vec3(-20.0, 0.0, 0.0);
	Points[2].ShapeIndex = 1;
	Points[2].EdgeIndex = 0;

	Points[3].Location = Vec3(40.0, 40.0, 0.0);

	const std::vector<bool> ToRemove = TautRope::GetPointsToRemove(Points, { Seam, Seam });

	CHECK(ToRemove[1]);
	CHECK(ToRemove[2]);
}
