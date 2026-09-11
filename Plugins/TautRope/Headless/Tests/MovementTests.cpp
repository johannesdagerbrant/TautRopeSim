// The vertex-fan movement solve: grouping points whose edges share a vertex,
// unfolding the fan to 2D, and placing every point on the straight line between
// the neighbouring anchors in one step.
//
// Every test states what it PROVES, then what it FIXES or GUARDS, and the
// sabotage that was seen to turn it red.
#include "Framework.h"

#include "TautRopeCore/Config.h"
#include "TautRopeCore/Movement.h"
#include "TautRopeCore/Point.h"

#include <cmath>
#include <vector>

using TautRope::Int2;
using TautRope::Quat;
using TautRope::Vec3;

namespace
{
	// A flat fan: three edges of length 100 radiating from vertex 0 in the XY
	// plane at 120, 90 and 60 degrees. Flat on purpose — unfolding a coplanar
	// fan is the identity, so the geodesic across it is the plain 3D straight
	// line between the anchors, which gives these tests an oracle that does not
	// re-implement the unfold.
	//
	// Edge 0 is deliberately listed as (outer, fan vertex): a grouping that
	// trusts the first edge's first endpoint instead of intersecting candidate
	// sets picks vertex 1 here and never forms the group.
	TautRope::CollisionShape MakeFlatFanShape()
	{
		TautRope::CollisionShape Shape;
		const double R = 100.0;
		Shape.Vertices = {
			Vec3(0.0, 0.0, 0.0),
			Vec3(R * std::cos(2.0943951023931953), R * std::sin(2.0943951023931953), 0.0),
			Vec3(0.0, R, 0.0),
			Vec3(R * std::cos(1.0471975511965976), R * std::sin(1.0471975511965976), 0.0),
		};
		Shape.Edges = {
			Int2(1, 0),
			Int2(0, 2),
			Int2(0, 3),
		};
		Shape.VertToEdges = { { 0, 1, 2 }, { 0 }, { 1 }, { 2 } };
		Shape.EdgeRotations = { Quat(), Quat(), Quat() };
		// The outer endpoints dangle (one adjacent edge), so only the fan vertex
		// can be crossed — which is what the snap test needs.
		Shape.IsCornerVertexList = { false, true, true, true };
		return Shape;
	}

	std::vector<TautRope::Point> MakeFanRope(
		const TautRope::CollisionShape& Shape
		, const Vec3& AnchorA
		, const Vec3& AnchorB
	)
	{
		std::vector<TautRope::Point> Points(5);
		Points[0].Location = AnchorA;
		Points[4].Location = AnchorB;
		for (int i = 0; i < 3; ++i)
		{
			TautRope::Point& P = Points[1 + i];
			const Int2& Edge = Shape.Edges[i];
			const int Outer = Edge.X == 0 ? Edge.Y : Edge.X;
			P.Location = Shape.Vertices[Outer] * 0.05;
			P.ShapeIndex = 0;
			P.EdgeIndex = i;
			P.Id = i;
		}
		return Points;
	}

	double DistanceToSegment(const Vec3& P, const Vec3& A, const Vec3& B)
	{
		const Vec3 AB = B - A;
		const double LenSq = AB.SizeSquared();
		double T = LenSq > 0.0 ? Vec3::Dot(P - A, AB) / LenSq : 0.0;
		T = T < 0.0 ? 0.0 : (T > 1.0 ? 1.0 : T);
		return (P - (A + AB * T)).Size();
	}
}

// PROVES: consecutive points on distinct edges sharing one vertex become one
// group, and the shared vertex comes from intersecting the candidate sets, not
// from the first edge's first endpoint.
// FIXES: the original GetMovementGroups fixed the group vertex to
// CandidateVerts[0] of the first point, so a first edge stored (outer, fan
// vertex) — edge 0 here — picked the outer endpoint and the run never grouped.
// Sabotage: take SharedVertCandidates[0] without narrowing and this goes red.
TEST(Movement_GroupsRunOfEdgesSharingOneVertex)
{
	const TautRope::CollisionShape Shape = MakeFlatFanShape();
	const std::vector<TautRope::Point> Points = MakeFanRope(
		Shape, Vec3(-120.0, 40.0, 0.0), Vec3(120.0, 40.0, 0.0));

	const std::vector<TautRope::MovementGroup> Groups =
		TautRope::GetMovementGroups(Points, { Shape });

	CHECK_EQ(Groups.size(), std::size_t(1));
	if (Groups.size() != 1)
	{
		return;
	}
	CHECK_EQ(Groups[0].VertIndex, 0);
	CHECK_EQ(Groups[0].FirstPointIndex, 1);
	CHECK_EQ(Groups[0].LastPointIndex, 3);
	CHECK_EQ(Groups[0].EdgeIndices.size(), std::size_t(3));
}

// PROVES: a run is split where the shape changes or where a point already sits
// on a vertex, so a fan never mixes shapes and never rides a crossing point.
// GUARDS: the same cross-shape confusion that once pruned a point a hundred
// units away because edge indices were compared across shapes
// (VertexCone_DoesNotGroupPointsFromDifferentShapes is the pruning-side twin).
// Sabotage: drop the same-shape requirement from the narrowing and this goes red.
TEST(Movement_GroupsSplitAcrossShapesAndVertexPoints)
{
	const TautRope::CollisionShape Shape = MakeFlatFanShape();

	std::vector<TautRope::Point> CrossShape = MakeFanRope(
		Shape, Vec3(-120.0, 40.0, 0.0), Vec3(120.0, 40.0, 0.0));
	// Same edge indices, second shape: indices match, identity does not.
	CrossShape[2].ShapeIndex = 1;
	const std::vector<TautRope::MovementGroup> AcrossShapes =
		TautRope::GetMovementGroups(CrossShape, { Shape, Shape });
	CHECK_EQ(AcrossShapes.size(), std::size_t(0));

	std::vector<TautRope::Point> OnVertex = MakeFanRope(
		Shape, Vec3(-120.0, 40.0, 0.0), Vec3(120.0, 40.0, 0.0));
	OnVertex[2].VertIndex = 0;
	const std::vector<TautRope::MovementGroup> AcrossVertexPoint =
		TautRope::GetMovementGroups(OnVertex, { Shape });
	CHECK_EQ(AcrossVertexPoint.size(), std::size_t(0));
}

// PROVES: the fan solve places every point of the group where the straight
// line between the anchors crosses its edge. On this coplanar fan the unfold
// is the identity, so the solved targets must be collinear with the anchors in
// 3D — an oracle independent of the unfold arithmetic.
// FIXES: the movement phase solving each point against its neighbours' current
// positions, which contracts a converging run by one neighbour-average step
// per frame. Recording 154909 measured that crawl at 0.33 units/frame over 615
// frames for one pair; the same replay with the fan solve finishes unwrapping
// at frame 863 instead of 1126.
// Sabotage: leave the anchor pivot out of the unfold (EdgeAngles[0] = 0) and
// this goes red with targets 30+ units off the line.
TEST(Movement_FanSolvePlacesPointsOnStraightLineAcrossFlatFan)
{
	const TautRope::CollisionShape Shape = MakeFlatFanShape();
	const Vec3 AnchorA(-120.0, 40.0, 0.0);
	const Vec3 AnchorB(120.0, 40.0, 0.0);
	std::vector<TautRope::Point> Points = MakeFanRope(Shape, AnchorA, AnchorB);

	const std::vector<TautRope::MovementGroup> Groups =
		TautRope::GetMovementGroups(Points, { Shape });
	CHECK_EQ(Groups.size(), std::size_t(1));
	if (Groups.size() != 1)
	{
		return;
	}

	std::vector<Vec3> Targets;
	for (const TautRope::Point& P : Points)
	{
		Targets.push_back(P.Location);
	}
	const bool bSolved = TautRope::SolveFanMovementGroup(Points, Targets, Groups[0], Shape);
	CHECK(bSolved);

	for (int i = 1; i <= 3; ++i)
	{
		const double OffLine = DistanceToSegment(Targets[i], AnchorA, AnchorB);
		CHECK(OffLine < 0.01);
		// Still riding its edge, not snapped to either end.
		CHECK_EQ(Points[i].VertIndex, TautRope::IndexNone);
		const double FromVert = Targets[i].Size();
		CHECK(FromVert > 1.0);
		CHECK(FromVert < 99.0);
		if (OffLine >= 0.01)
		{
			std::printf("      point %d target off the anchor line by %.3f\n", i, OffLine);
		}
	}
}

// PROVES: when the unfolded straight line passes on the vertex side of the fan
// — here below the vertex while every edge points up — each point snaps into
// the vertex-crossing state at the shared vertex instead of staying partway
// along an edge the taut rope has left.
// GUARDS: the taut behaviour at a vertex: the rope pinches onto it, which is
// what lets the pruning phase remove the converged group and the rope slide
// over. Sabotage: on a miss, clamp to the edge instead of crossing, and this
// goes red with all three points still on their edges.
TEST(Movement_FanSolveSnapsMissedEdgesToTheSharedVertex)
{
	const TautRope::CollisionShape Shape = MakeFlatFanShape();
	const Vec3 AnchorA(-120.0, -20.0, 0.0);
	const Vec3 AnchorB(120.0, -20.0, 0.0);
	std::vector<TautRope::Point> Points = MakeFanRope(Shape, AnchorA, AnchorB);

	const std::vector<TautRope::MovementGroup> Groups =
		TautRope::GetMovementGroups(Points, { Shape });
	CHECK_EQ(Groups.size(), std::size_t(1));
	if (Groups.size() != 1)
	{
		return;
	}

	std::vector<Vec3> Targets;
	for (const TautRope::Point& P : Points)
	{
		Targets.push_back(P.Location);
	}
	const bool bSolved = TautRope::SolveFanMovementGroup(Points, Targets, Groups[0], Shape);
	CHECK(bSolved);

	for (int i = 1; i <= 3; ++i)
	{
		CHECK_EQ(Points[i].VertIndex, 0);
	}
}
