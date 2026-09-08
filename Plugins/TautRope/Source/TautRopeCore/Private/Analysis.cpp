#include "TautRopeCore/Analysis.h"

#include "TautRopeCore/Collision.h"

#include <algorithm>
#include <cmath>

namespace TautRope
{
	namespace
	{
		bool IsNewPlane(const ShapePlanes& Planes, const Vec3& Point, const Vec3& Normal)
		{
			for (std::size_t i = 0; i < Planes.Normals.size(); ++i)
			{
				if (Vec3::Dot(Planes.Normals[i], Normal) > 0.999
					&& Math::Abs(Vec3::Dot(Planes.Normals[i], Point - Planes.Points[i])) < 0.01)
				{
					return false;
				}
			}
			return true;
		}
	}

	ShapePlanes FindShapePlanes(const CollisionShape& Shape, double Tolerance)
	{
		ShapePlanes Result;
		for (int32 EdgeIndex = 0; EdgeIndex < Num(Shape.Edges); ++EdgeIndex)
		{
			const Int2& Edge = Shape.Edges[EdgeIndex];
			const Vec3 Midpoint = (Shape.Vertices[Edge.X] + Shape.Vertices[Edge.Y]) * 0.5;
			const Vec3 Normal = Shape.EdgeRotations[EdgeIndex].GetUpVector().GetSafeNormal();
			if (Normal.SizeSquared() <= 0.0)
			{
				continue;
			}

			// Supporting plane test: no vertex may lie outside it.
			//
			// That alone does not identify a face, because the plane through a real
			// silhouette edge also supports the hull -- its normal bisects the two
			// adjacent faces, so the plane is a bevel touching the hull along that
			// edge. What separates the two is how many vertices lie ON the plane: a
			// face contains all of its own corners, a silhouette edge only its two
			// endpoints.
			bool bSupports = true;
			int32 OnPlane = 0;
			for (const Vec3& V : Shape.Vertices)
			{
				const double Signed = Vec3::Dot(V - Midpoint, Normal);
				if (Signed > Tolerance)
				{
					bSupports = false;
					break;
				}
				if (Math::Abs(Signed) <= Tolerance)
				{
					++OnPlane;
				}
			}
			if (!bSupports || OnPlane < 3)
			{
				continue;
			}

			Result.InFaceEdges.push_back(EdgeIndex);
			if (IsNewPlane(Result, Midpoint, Normal))
			{
				Result.Points.push_back(Midpoint);
				Result.Normals.push_back(Normal);
			}
		}
		return Result;
	}

	// Every plane is pushed inward by SurfaceTolerance before testing, so the hull
	// used here is slightly smaller than the real one.
	//
	// This matters more than it looks. Rope points rest ON the surface by design,
	// which puts them exactly in a face plane, and a segment between two points of
	// the same face lies exactly in that plane. Without a tolerance the answer for
	// that case flips between zero and the full width of the face depending on
	// whether rounding puts a point a fraction inside or outside -- and the rope
	// lying flat against a face is correct behaviour, not penetration. Only
	// crossing deeper than the tolerance counts.
	static constexpr double SurfaceTolerance = 0.05;

	double PointPenetrationDepth(const ShapePlanes& Planes, const Vec3& Point)
	{
		if (!Planes.IsUsable())
		{
			return 0.0;
		}
		double Deepest = -1.0;
		for (std::size_t i = 0; i < Planes.Normals.size(); ++i)
		{
			const double Signed = Vec3::Dot(Point - Planes.Points[i], Planes.Normals[i]) + SurfaceTolerance;
			if (Signed >= 0.0)
			{
				return 0.0; // outside this face, so outside the hull
			}
			const double Depth = -Signed;
			if (Deepest < 0.0 || Depth < Deepest)
			{
				Deepest = Depth;
			}
		}
		return Deepest < 0.0 ? 0.0 : Deepest;
	}

	double SegmentInsideLength(const ShapePlanes& Planes, const Vec3& A, const Vec3& B)
	{
		if (!Planes.IsUsable())
		{
			return 0.0;
		}
		const Vec3 Dir = B - A;
		double Low = 0.0;
		double High = 1.0;
		for (std::size_t i = 0; i < Planes.Normals.size(); ++i)
		{
			const double Offset = Vec3::Dot(A - Planes.Points[i], Planes.Normals[i]) + SurfaceTolerance;
			const double Slope = Vec3::Dot(Dir, Planes.Normals[i]);
			if (Math::Abs(Slope) <= 0.0)
			{
				if (Offset >= 0.0)
				{
					return 0.0;
				}
				continue;
			}
			const double Crossing = -Offset / Slope;
			if (Slope > 0.0)
			{
				if (Crossing < High) { High = Crossing; }
			}
			else
			{
				if (Crossing > Low) { Low = Crossing; }
			}
			if (Low >= High)
			{
				return 0.0;
			}
		}
		return (High - Low) * Dir.Size();
	}

	double SweepConditioning(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const Vec3& LineA
		, const Vec3& LineB
	)
	{
		const Vec3 Dir = LineB - LineA;
		const Vec3 Edge1 = ToCorner - FromCorner;
		const Vec3 Edge2 = SupportCorner - FromCorner;
		const double Det = Vec3::Dot(Edge1, Vec3::Cross(Dir, Edge2));
		const double Scale = Edge1.Size() * Dir.Size() * Edge2.Size();
		return Scale > 0.0 ? Math::Abs(Det) / Scale : 0.0;
	}

	PenetrationReport AnalysePenetration(const Recording& InRecording, double Tolerance)
	{
		PenetrationReport Report;
		std::vector<ShapePlanes> Planes;
		Planes.reserve(InRecording.Shapes.size());
		for (const CollisionShape& Shape : InRecording.Shapes)
		{
			Planes.push_back(FindShapePlanes(Shape));
			if (!Planes.back().IsUsable())
			{
				Report.bAnyShapeUnusable = true;
			}
		}

		Report.FramesCompared = Num(InRecording.Frames);
		for (int32 FrameIndex = 0; FrameIndex < Num(InRecording.Frames); ++FrameIndex
			)
		{
			const std::vector<RecordedPoint>& Points = InRecording.Frames[FrameIndex].Capture.AfterPruning;
			double FrameWorst = 0.0;

			for (int32 i = 0; i + 1 < Num(Points); ++i)
			{
				for (int32 s = 0; s < Num(Planes); ++s)
				{
					const double Length = SegmentInsideLength(Planes[s], Points[i].Location, Points[i + 1].Location);
					if (Length > FrameWorst) { FrameWorst = Length; }
					if (Length > Report.PeakSegmentLength)
					{
						Report.PeakSegmentLength = Length;
						Report.PeakSegmentFrame = FrameIndex;
						Report.PeakSegmentShape = s;
						Report.PeakSegmentPointA = Points[i].Id;
						Report.PeakSegmentPointB = Points[i + 1].Id;
					}
				}
			}
			for (const RecordedPoint& P : Points)
			{
				for (int32 s = 0; s < Num(Planes); ++s)
				{
					const double Depth = PointPenetrationDepth(Planes[s], P.Location);
					if (Depth > Report.PeakPointDepth)
					{
						Report.PeakPointDepth = Depth;
						Report.PeakPointFrame = FrameIndex;
						Report.PeakPointId = P.Id;
					}
				}
			}

			if (FrameWorst > Tolerance)
			{
				++Report.FramesAffected;
				if (Report.FirstFrame == IndexNone)
				{
					Report.FirstFrame = FrameIndex;
				}
			}
			if (FrameIndex == Num(InRecording.Frames) - 1)
			{
				Report.FinalSegmentLength = FrameWorst;
			}
		}
		return Report;
	}

	EdgeUsageReport AnalyseEdgeUsage(const Recording& InRecording)
	{
		EdgeUsageReport Report;
		std::vector<std::vector<bool>> InFace;
		for (const CollisionShape& Shape : InRecording.Shapes)
		{
			const ShapePlanes Planes = FindShapePlanes(Shape);
			std::vector<bool> Flags(Shape.Edges.size(), false);
			for (const int32 EdgeIndex : Planes.InFaceEdges)
			{
				Flags[static_cast<std::size_t>(EdgeIndex)] = true;
			}
			Report.TotalEdges += Num(Shape.Edges);
			Report.InFaceEdges += Num(Planes.InFaceEdges);
			InFace.push_back(Flags);
		}

		for (int32 FrameIndex = 0; FrameIndex < Num(InRecording.Frames); ++FrameIndex)
		{
			for (const RecordedPoint& P : InRecording.Frames[FrameIndex].Capture.AfterPruning)
			{
				if (P.ShapeIndex < 0 || P.EdgeIndex < 0 || P.ShapeIndex >= Num(InFace))
				{
					continue;
				}
				++Report.AttachedPointFrames;
				if (InFace[P.ShapeIndex][static_cast<std::size_t>(P.EdgeIndex)])
				{
					++Report.PointFramesOnInFaceEdges;
					if (Report.FirstFrameOnInFaceEdge == IndexNone)
					{
						Report.FirstFrameOnInFaceEdge = FrameIndex;
						Report.FirstPointIdOnInFaceEdge = P.Id;
						Report.FirstInFaceEdgeIndex = P.EdgeIndex;
						Report.FirstInFaceShapeIndex = P.ShapeIndex;
					}
				}
			}
		}
		return Report;
	}

	std::vector<VertexApproach> AnalyseVertexApproaches(const Recording& InRecording, int32 MaxResults)
	{
		struct Track
		{
			VertexApproach Approach;
			std::vector<double> Distances;
			Vec3 LastA;
			Vec3 LastB;
			double TailTravel = 0.0;
			int32 TailFrames = 0;
		};
		std::vector<Track> Tracks;

		for (int32 FrameIndex = 0; FrameIndex < Num(InRecording.Frames); ++FrameIndex)
		{
			const std::vector<RecordedPoint>& Points = InRecording.Frames[FrameIndex].Capture.AfterPruning;
			for (int32 i = 0; i + 1 < Num(Points); ++i)
			{
				const RecordedPoint& A = Points[i];
				const RecordedPoint& B = Points[i + 1];
				if (A.ShapeIndex < 0 || A.ShapeIndex != B.ShapeIndex || A.EdgeIndex == B.EdgeIndex
					|| A.EdgeIndex < 0 || B.EdgeIndex < 0)
				{
					continue;
				}
				const CollisionShape& Shape = InRecording.Shapes[A.ShapeIndex];
				const Int2& EA = Shape.Edges[A.EdgeIndex];
				const Int2& EB = Shape.Edges[B.EdgeIndex];
				int32 Shared = IndexNone;
				if (EA.X == EB.X || EA.X == EB.Y) { Shared = EA.X; }
				else if (EA.Y == EB.X || EA.Y == EB.Y) { Shared = EA.Y; }
				if (Shared == IndexNone)
				{
					continue;
				}

				const Vec3& V = Shape.Vertices[Shared];
				const double Distance = std::max(Vec3::Dist(A.Location, V), Vec3::Dist(B.Location, V));

				Track* Found = nullptr;
				for (Track& T : Tracks)
				{
					if (T.Approach.PointIdA == A.Id && T.Approach.PointIdB == B.Id
						&& T.Approach.VertIndex == Shared && T.Approach.ShapeIndex == A.ShapeIndex)
					{
						Found = &T;
						break;
					}
				}
				if (Found == nullptr)
				{
					Track T;
					T.Approach.FirstFrame = FrameIndex;
					T.Approach.PointIdA = A.Id;
					T.Approach.PointIdB = B.Id;
					T.Approach.ShapeIndex = A.ShapeIndex;
					T.Approach.VertIndex = Shared;
					T.Approach.StartDistance = Distance;
					Tracks.push_back(T);
					Found = &Tracks.back();
				}
				else
				{
					Found->TailTravel += Vec3::Dist(A.Location, Found->LastA);
					++Found->TailFrames;
				}
				Found->Approach.LastFrame = FrameIndex;
				Found->Approach.EndDistance = Distance;
				Found->LastA = A.Location;
				Found->LastB = B.Location;
				Found->Distances.push_back(Distance);
			}
		}

		std::vector<VertexApproach> Out;
		for (Track& T : Tracks)
		{
			if (T.Approach.LastFrame - T.Approach.FirstFrame < 30)
			{
				continue;
			}
			// Speed over the tail of the track, which is where the stall shows.
			const std::size_t Tail = std::min<std::size_t>(200, T.Distances.size());
			if (Tail >= 2)
			{
				const double Closed = T.Distances[T.Distances.size() - Tail] - T.Distances.back();
				T.Approach.FinalSpeed = Closed / static_cast<double>(Tail - 1);
			}
			T.Approach.FramesToArrive = T.Approach.FinalSpeed > 0.0
				? T.Approach.EndDistance / T.Approach.FinalSpeed
				: 0.0;
			Out.push_back(T.Approach);
		}
		std::sort(Out.begin(), Out.end(), [](const VertexApproach& L, const VertexApproach& R)
			{
				return (L.LastFrame - L.FirstFrame) > (R.LastFrame - R.FirstFrame);
			});
		if (Num(Out) > MaxResults)
		{
			Out.resize(static_cast<std::size_t>(MaxResults));
		}
		return Out;
	}

	ConditioningReport AnalyseConditioning(const Recording& InRecording, double CoplanarThreshold)
	{
		ConditioningReport Report;
		for (const RecordedFrame& Frame : InRecording.Frames)
		{
			const std::vector<RecordedPoint>& Before = Frame.Capture.AfterMovement;
			const std::vector<RecordedPoint>& After = Frame.Capture.AfterCollision;

			for (int32 i = 0; i + 1 < Num(Before); ++i)
			{
				// Match by id: the collision phase may have inserted points.
				const RecordedPoint* TargetA = nullptr;
				const RecordedPoint* TargetB = nullptr;
				for (const RecordedPoint& P : After)
				{
					if (P.Id == Before[i].Id) { TargetA = &P; }
					if (P.Id == Before[i + 1].Id) { TargetB = &P; }
				}
				if (TargetA == nullptr || TargetB == nullptr)
				{
					continue;
				}

				const Vec3 Corners[2][3] = {
					{ Before[i].Location, TargetA->Location, Before[i + 1].Location },
					{ Before[i + 1].Location, TargetB->Location, TargetA->Location },
				};

				for (int32 Sweep = 0; Sweep < 2; ++Sweep)
				{
					for (int32 s = 0; s < Num(InRecording.Shapes); ++s)
					{
						const CollisionShape& Shape = InRecording.Shapes[s];
						for (int32 e = 0; e < Num(Shape.Edges); ++e)
						{
							const Int2& Edge = Shape.Edges[e];
							Vec3 Location;
							Vec3 OnSweepEdge;
							float Ratio = MaxFloat;
							const double Conditioning = SweepConditioning(
								Corners[Sweep][0], Corners[Sweep][1], Corners[Sweep][2]
								, Shape.Vertices[Edge.X], Shape.Vertices[Edge.Y]);
							const bool bHit = GetTriangleLineIntersection(
								Corners[Sweep][0], Corners[Sweep][1], Corners[Sweep][2]
								, Shape.Vertices[Edge.X], Shape.Vertices[Edge.Y]
								, Location, OnSweepEdge, Ratio);

							++Report.Tests;
							if (Conditioning < CoplanarThreshold)
							{
								++Report.NearCoplanar;
								if (bHit) { ++Report.NearCoplanarAccepted; }
							}
							else if (bHit)
							{
								++Report.WellConditionedAccepted;
							}
						}
					}
				}
			}
		}
		return Report;
	}
}
