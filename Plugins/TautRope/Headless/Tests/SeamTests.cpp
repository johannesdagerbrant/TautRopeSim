// Seam welding: splitting edges at foreign vertices that lie on them, so
// overlapping cross-shape seam edges become 1:1 coincident segments.
#include "Framework.h"

#include "TautRopeCore/Seam.h"

#include <vector>

using TautRope::Int2;
using TautRope::Quat;
using TautRope::Vec3;

namespace
{
	TautRope::CollisionShape MakeEdgeShape(const Vec3& A, const Vec3& B)
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = { A, B };
		Shape.Edges = { Int2(0, 1) };
		Shape.VertToEdges = { { 0 }, { 0 } };
		Shape.EdgeRotations = { Quat(0.1, 0.2, 0.3, 0.9) };
		Shape.IsCornerVertexList = { true, true };
		return Shape;
	}
}

// PROVES: an edge carrying another shape's vertex in its interior is split
// there - the first half keeps its index, the second half and the junction
// vertex are appended, and the junction takes the foreign vertex's exact
// coordinates and is crossable.
// FIXES: the cross-shape seam junctions on the quartered cone (recording
// 085126): one hull's vertex sat mid-edge on the other's overlapping seam
// edge, so a point sliding along it crossed a junction no vertex machinery
// knew about, and the collision sweep exploded the rope to 151,986 points.
// Sabotage: make WeldSeamsAcrossShapes return without splitting and this goes
// red on the edge count.
TEST(Seam_SplitsEdgeAtForeignVertexOnIt)
{
	std::vector<TautRope::CollisionShape> Shapes = {
		MakeEdgeShape(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0)),
		MakeEdgeShape(Vec3(40.0, 0.0, 0.0), Vec3(40.0, 50.0, 0.0)),
	};

	TautRope::WeldSeamsAcrossShapes(Shapes);

	const TautRope::CollisionShape& A = Shapes[0];
	CHECK_EQ(A.Edges.size(), std::size_t(2));
	CHECK_EQ(A.Vertices.size(), std::size_t(3));
	if (A.Edges.size() != 2)
	{
		return;
	}
	// First half keeps index 0 and its A-side endpoint; second half is appended.
	CHECK_EQ(A.Edges[0].X, 0);
	CHECK_EQ(A.Edges[0].Y, 2);
	CHECK_EQ(A.Edges[1].X, 2);
	CHECK_EQ(A.Edges[1].Y, 1);
	// The junction is the foreign vertex, bit for bit, and is crossable.
	CHECK_BITS(A.Vertices[2].X, 40.0);
	CHECK_BITS(A.Vertices[2].Y, 0.0);
	CHECK(!A.IsCornerVertexList[2]);
	CHECK_EQ(A.VertToEdges[2].size(), std::size_t(2));
	// The far endpoint now belongs to the appended half, not the original.
	CHECK_EQ(A.VertToEdges[1][0], 1);
	// Both halves keep the original rotation: same line, same orientation.
	CHECK_BITS(A.EdgeRotations[1].Z, A.EdgeRotations[0].Z);

	// Shape B is untouched: no vertex of A lies in the interior of its edge.
	CHECK_EQ(Shapes[1].Edges.size(), std::size_t(1));
}

// PROVES: welding an already 1:1 seam - both shapes carrying the same segment
// with matching endpoints - changes nothing.
// GUARDS: the fixpoint loop from splitting at endpoint-coincident vertices
// forever; endpoints are excluded by tolerance, and a weld that split there
// would never terminate. Sabotage: drop the endpoint exclusion from the
// interior test and this hangs or reds on the edge count.
TEST(Seam_LeavesExactTwinSegmentsAlone)
{
	std::vector<TautRope::CollisionShape> Shapes = {
		MakeEdgeShape(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0)),
		MakeEdgeShape(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0)),
	};

	TautRope::WeldSeamsAcrossShapes(Shapes);

	CHECK_EQ(Shapes[0].Edges.size(), std::size_t(1));
	CHECK_EQ(Shapes[1].Edges.size(), std::size_t(1));
}
