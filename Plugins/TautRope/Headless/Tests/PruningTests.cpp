// Minimal cases against Pruning.h.

#include "Framework.h"

#include "TautRopeCore/Collision.h"
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
