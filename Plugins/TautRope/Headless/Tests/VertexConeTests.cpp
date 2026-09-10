// Minimal cases against VertexHandling.h.

#include "Framework.h"

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Point.h"
#include "TautRopeCore/VertexHandling.h"

#include <cstdio>
#include <vector>

using TautRope::Vec3;

namespace
{
	// Two edges meeting at vertex 0, so vertex 0 has a cone.
	TautRope::CollisionShape MakeConeShape()
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = { Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0) };
		Shape.Edges = { TautRope::Int2(0, 1), TautRope::Int2(0, 2) };
		Shape.VertToEdges = { { 0, 1 }, { 0 }, { 1 } };
		Shape.EdgeRotations = { TautRope::Quat(), TautRope::Quat() };
		Shape.IsCornerVertexList = { true, false, false };
		return Shape;
	}

	// A second shape a hundred units away that shares nothing with the first except
	// the numbers used to index its own vertices and edges.
	TautRope::CollisionShape MakeDistantShape()
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = { Vec3(100.0, 0.0, 0.0), Vec3(101.0, 0.0, 0.0) };
		Shape.Edges = { TautRope::Int2(0, 1) };
		Shape.VertToEdges = { { 0 }, { 0 } };
		Shape.EdgeRotations = { TautRope::Quat() };
		Shape.IsCornerVertexList = { false, false };
		return Shape;
	}

	TautRope::Point MakePoint(const Vec3& Location, const int32_t Shape, const int32_t Edge, const int32_t Vert, const int32_t Id)
	{
		TautRope::Point P;
		P.Location = Location;
		P.ShapeIndex = static_cast<TautRope::int32>(Shape);
		P.EdgeIndex = static_cast<TautRope::int32>(Edge);
		P.VertIndex = static_cast<TautRope::int32>(Vert);
		P.Id = static_cast<TautRope::int32>(Id);
		return P;
	}
}

// PROVES: the vertex cone rule groups a point on one shape together with a point
// on a completely different shape, purely because their edge numbers match.
// FIXES: the cross-shape penetration in recording 105002. At frame 1043 the
// pruning phase removed points 4/13, 4/0 and 0/13 in one go and put 98 units of
// rope inside shape 0. --removals 1043 attributes all three to this rule, and
// none of them are on a flat edge. Shape 4 edge 13 and shape 0 edge 13 share an
// index and nothing else: GetAdjacentPointsOnSameVertexCone reads VertToEdges
// from the vertex point's own shape and then compares those edge indices against
// neighbours belonging to any shape at all.
TEST(VertexCone_DoesNotGroupPointsFromDifferentShapes)
{
	const std::vector<TautRope::CollisionShape> Shapes = { MakeConeShape(), MakeDistantShape() };

	// A foreign point on EACH side of the vertex point. The group is expanded by
	// two separate loops, one walking back and one walking forward, and each needs
	// its own shape check. Removing the check from only the backward loop left the
	// earlier version of this test green while the bug was still there.
	const std::vector<TautRope::Point> RopePoints = {
		MakePoint(Vec3(0.0, -1.0, 0.0), -1, -1, -1, 0)      // free end
		, MakePoint(Vec3(99.5, 0.0, 0.0), 1, 0, -1, 1)      // on shape 1, before
		, MakePoint(Vec3(0.0, 0.0, 0.0), 0, 0, 0, 2)        // on shape 0, at vertex 0
		, MakePoint(Vec3(100.5, 0.0, 0.0), 1, 0, -1, 3)     // on shape 1, after
	};

	const std::vector<bool> ToRemove = TautRope::GetAdjacentPointsOnSameVertexCone(RopePoints, Shapes);

	std::printf("      remove flags: %d %d %d %d\n",
		ToRemove[0] ? 1 : 0, ToRemove[1] ? 1 : 0, ToRemove[2] ? 1 : 0, ToRemove[3] ? 1 : 0);

	// The vertex point itself is in a cone and may go.
	CHECK(ToRemove[2]);

	// Neither point on the other shape has anything to do with that cone; they are
	// a hundred units away and share only the numbers used to index them.
	CHECK(!ToRemove[1]);
	CHECK(!ToRemove[3]);
}
