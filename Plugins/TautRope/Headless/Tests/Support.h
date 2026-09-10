#pragma once

#include "TautRopeCore/Analysis.h"
#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Math.h"
#include "TautRopeCore/Point.h"

#include <vector>

// Fixtures and measurement helpers shared by the simulation tests.
namespace TautRopeTest
{
	using TautRope::Int2;
	using TautRope::Quat;
	using TautRope::Vec3;

	// A real shape, captured from TestLevel1 rather than reconstructed here. The
	// edge rotations come out of UE's FRotationMatrix::MakeFromXZ, and
	// reimplementing that in test code would risk a fixture that quietly differs
	// from what the simulation actually sees. 18 edges rather than 12 because the
	// convex hull is triangulated, so each face contributes a diagonal.
	inline TautRope::CollisionShape MakeCapturedBoxShape()
	{
		TautRope::CollisionShape Shape;
		Shape.Vertices = {
			Vec3(-513.3726267384086, 314.58725399999997, 334.35483133103691),
			Vec3(-413.37262673840866, 314.58725399999997, 334.35483133103691),
			Vec3(-413.37262673840866, 414.58725399999997, 334.35483133103691),
			Vec3(-513.3726267384086, 414.58725399999997, 334.35483133103691),
			Vec3(-513.3726267384086, 314.58725399999997, -9.9822566317395172),
			Vec3(-513.3726267384086, 414.58725399999997, -9.9822566317395172),
			Vec3(-413.37262673840866, 414.58725399999997, -9.9822566317395172),
			Vec3(-413.37262673840866, 314.58725399999997, -9.9822566317395172),
		};
		Shape.Edges = {
			Int2(0, 1),
			Int2(1, 2),
			Int2(0, 2),
			Int2(2, 3),
			Int2(0, 3),
			Int2(4, 5),
			Int2(5, 6),
			Int2(4, 6),
			Int2(6, 7),
			Int2(4, 7),
			Int2(2, 6),
			Int2(2, 7),
			Int2(1, 7),
			Int2(0, 4),
			Int2(3, 4),
			Int2(3, 5),
			Int2(2, 5),
			Int2(1, 4),
		};
		Shape.EdgeRotations = {
			Quat(0.38268343236508984, 0, 0, 0.92387953251128674),
			Quat(0.27059805007309851, 0.27059805007309851, 0.65328148243818829, 0.65328148243818829),
			Quat(0, 0, 0.38268343236508995, 0.92387953251128674),
			Quat(0, 0.38268343236508984, 0.92387953251128674, 0),
			Quat(-0.27059805007309851, -0.27059805007309851, 0.65328148243818829, 0.65328148243818829),
			Quat(0.65328148243818829, 0.65328148243818829, -0.27059805007309851, -0.27059805007309851),
			Quat(0.92387953251128674, 0, 0, -0.38268343236508984),
			Quat(0.92387953251128674, 0.38268343236508995, 0, 0),
			Quat(0.65328148243818829, -0.65328148243818829, 0.27059805007309851, -0.27059805007309851),
			Quat(0.92387953251128674, 0, 0, 0.38268343236508984),
			Quat(-0.27059805007309851, 0.65328148243818829, 0.27059805007309851, 0.65328148243818829),
			Quat(-0.099595495660723246, 0.70005766708471584, -0.099595495660723246, 0.70005766708471584),
			Quat(0.27059805007309851, 0.65328148243818829, -0.27059805007309851, 0.65328148243818829),
			Quat(0.65328148243818829, 0.27059805007309851, -0.65328148243818829, 0.27059805007309851),
			Quat(0.70005766708471584, -0.099595495660723246, -0.70005766708471584, 0.099595495660723246),
			Quat(0.65328148243818829, -0.27059805007309851, -0.65328148243818829, -0.27059805007309851),
			Quat(-0.42459087325990441, 0.5654401739745698, 0.56544017397456992, 0.42459087325990441),
			Quat(0.42459087325990441, 0.5654401739745698, -0.56544017397456992, 0.42459087325990441),
		};
		Shape.VertToEdges = {
			{ 0, 2, 4, 13 },
			{ 0, 1, 12, 17 },
			{ 1, 2, 3, 10, 11, 16 },
			{ 3, 4, 14, 15 },
			{ 5, 7, 9, 13, 14, 17 },
			{ 5, 6, 15, 16 },
			{ 6, 7, 8, 10 },
			{ 8, 9, 11, 12 },
		};
		Shape.IsCornerVertexList = { false, false, false, false, false, false, false, false };
		return Shape;
	}

	struct Bounds
	{
		Vec3 Min;
		Vec3 Max;
	};

	inline Bounds ShapeBounds(const TautRope::CollisionShape& Shape)
	{
		Bounds Result;
		Result.Min = Shape.Vertices[0];
		Result.Max = Shape.Vertices[0];
		for (const Vec3& V : Shape.Vertices)
		{
			if (V.X < Result.Min.X) { Result.Min.X = V.X; }
			if (V.Y < Result.Min.Y) { Result.Min.Y = V.Y; }
			if (V.Z < Result.Min.Z) { Result.Min.Z = V.Z; }
			if (V.X > Result.Max.X) { Result.Max.X = V.X; }
			if (V.Y > Result.Max.Y) { Result.Max.Y = V.Y; }
			if (V.Z > Result.Max.Z) { Result.Max.Z = V.Z; }
		}
		return Result;
	}

	// How far inside the shape a point sits, 0 when outside. The captured shape is
	// axis aligned, so its bounding box is the shape.
	// Penetration comes from core, not from a bounds test written here. An
	// axis-aligned box approximation of the hull would quietly disagree with the
	// simulation about where the surface is, and mirroring production geometry in
	// a test is how measurements end up describing something else.
	inline const TautRope::ShapePlanes& PlanesFor(const TautRope::CollisionShape& Shape)
	{
		static TautRope::ShapePlanes Planes = TautRope::FindShapePlanes(Shape);
		return Planes;
	}

	// Deepest any single rope POINT sits inside the shape.
	inline double DeepestPenetration(const std::vector<TautRope::Point>& Points, const TautRope::CollisionShape& Shape)
	{
		const TautRope::ShapePlanes& Planes = PlanesFor(Shape);
		double Deepest = 0.0;
		for (const TautRope::Point& P : Points)
		{
			const double Depth = TautRope::PointPenetrationDepth(Planes, P.Location);
			if (Depth > Deepest) { Deepest = Depth; }
		}
		return Deepest;
	}

	// Longest run of rope LINE inside the shape. This is the one that matters: the
	// defect leaves both endpoints resting on the surface and drives the segment
	// between them through the solid, so a point-only check cannot see it.
	inline double DeepestSegmentInside(const std::vector<TautRope::Point>& Points, const TautRope::CollisionShape& Shape)
	{
		const TautRope::ShapePlanes& Planes = PlanesFor(Shape);
		double Worst = 0.0;
		for (std::size_t i = 0; i + 1 < Points.size(); ++i)
		{
			const double Inside = TautRope::SegmentInsideLength(Planes, Points[i].Location, Points[i + 1].Location);
			if (Inside > Worst) { Worst = Inside; }
		}
		return Worst;
	}

	inline double RopeLength(const std::vector<TautRope::Point>& Points)
	{
		double Length = 0.0;
		for (std::size_t i = 0; i + 1 < Points.size(); ++i)
		{
			Length += Vec3::Dist(Points[i].Location, Points[i + 1].Location);
		}
		return Length;
	}
}
