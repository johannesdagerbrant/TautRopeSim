#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"
#include "TautRopeCore/Recording.h"

#include <vector>

// Measurements over a recording. These live in core rather than in the replay
// CLI so the unit tests can assert on the same numbers an agent reads, and so
// they run at compiled speed -- the alternative, ad-hoc scripting over the text
// format, is thousands of times slower than the loop it serves.
namespace TautRope
{
	// A convex hull's face planes, recovered from what a shape actually stores.
	//
	// Each edge carries a rotation whose up vector is the average of its adjacent
	// face normals. For an edge in the middle of a flat face -- the diagonal every
	// quad face gets when triangulated -- both adjacent triangles are coplanar, so
	// that average IS the face normal and the plane through the edge supports the
	// whole hull. For a real silhouette edge the average points between two faces
	// and the plane cuts through. Testing which is which therefore identifies the
	// face planes and the in-face edges at the same time.
	struct TAUTROPE_CORE_API ShapePlanes
	{
		std::vector<Vec3> Points;
		std::vector<Vec3> Normals;

		// Edges whose plane supports the hull, i.e. edges lying flat across a face.
		// A rope point should never rest on one.
		std::vector<int32> InFaceEdges;

		// Fewer than four planes cannot bound a volume, so the penetration numbers
		// below are not trustworthy for this shape.
		bool IsUsable() const { return Points.size() >= 4; }
	};

	TAUTROPE_CORE_API ShapePlanes FindShapePlanes(const CollisionShape& Shape, double Tolerance = 0.05);

	// How far inside the hull the point sits; 0 when outside.
	TAUTROPE_CORE_API double PointPenetrationDepth(const ShapePlanes& Planes, const Vec3& Point);

	// Length of segment AB lying inside the hull. This is what "the rope line
	// intersects the shape" means: both endpoints can sit on the surface while the
	// line between them cuts straight through.
	TAUTROPE_CORE_API double SegmentInsideLength(const ShapePlanes& Planes, const Vec3& A, const Vec3& B);

	// |Det| normalised by the three edge lengths, as GetTriangleLineIntersection
	// sees it. Near zero means the sweep triangle and the edge are coplanar, which
	// that routine cannot solve. Dimensionless, so it is comparable across scales.
	TAUTROPE_CORE_API double SweepConditioning(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const Vec3& LineA
		, const Vec3& LineB
	);

	struct TAUTROPE_CORE_API PenetrationReport
	{
		int32 FirstFrame = IndexNone;
		int32 FramesAffected = 0;
		int32 FramesCompared = 0;

		double PeakSegmentLength = 0.0;
		int32 PeakSegmentFrame = IndexNone;
		int32 PeakSegmentShape = IndexNone;
		int32 PeakSegmentPointA = IndexNone;
		int32 PeakSegmentPointB = IndexNone;

		double PeakPointDepth = 0.0;
		int32 PeakPointFrame = IndexNone;
		int32 PeakPointId = IndexNone;

		double FinalSegmentLength = 0.0;
		bool bAnyShapeUnusable = false;
	};

	TAUTROPE_CORE_API PenetrationReport AnalysePenetration(const Recording& InRecording, double Tolerance = 0.5);

	struct TAUTROPE_CORE_API EdgeUsageReport
	{
		int32 TotalEdges = 0;
		int32 InFaceEdges = 0;
		int32 AttachedPointFrames = 0;
		int32 PointFramesOnInFaceEdges = 0;
		int32 FirstFrameOnInFaceEdge = IndexNone;
		int32 FirstPointIdOnInFaceEdge = IndexNone;
		int32 FirstInFaceEdgeIndex = IndexNone;
		int32 FirstInFaceShapeIndex = IndexNone;

		// Where points come from. A point BORN attached to an in-face edge was put
		// there by the collision phase, which means the sweep that created it was
		// not coplanar with that edge -- the rope was still approaching the face
		// from off-plane. A point that instead SLID onto an in-face edge got there
		// later. The two need different fixes, so they are counted separately.
		int32 PointsBorn = 0;
		int32 PointsBornOnInFaceEdge = 0;
		int32 PointsBornOnSameFrameAsAnother = 0;
		int32 FirstBornOnInFaceFrame = IndexNone;
		int32 FirstBornOnInFacePointId = IndexNone;
	};

	TAUTROPE_CORE_API EdgeUsageReport AnalyseEdgeUsage(const Recording& InRecording);

	// Two adjacent points on edges that meet at a vertex, tracked as they close on
	// it. This is how the convergence slowdown is measured.
	struct TAUTROPE_CORE_API VertexApproach
	{
		int32 FirstFrame = IndexNone;
		int32 LastFrame = IndexNone;
		int32 PointIdA = IndexNone;
		int32 PointIdB = IndexNone;
		int32 ShapeIndex = IndexNone;
		int32 VertIndex = IndexNone;

		double StartDistance = 0.0;
		double EndDistance = 0.0;
		double FinalSpeed = 0.0;      // units per frame, averaged over the tail
		double FramesToArrive = 0.0;  // at FinalSpeed; infinite in effect if it stalls
	};

	// The pair that gets closest to a shared vertex and stays there longest.
	TAUTROPE_CORE_API std::vector<VertexApproach> AnalyseVertexApproaches(const Recording& InRecording, int32 MaxResults = 5);

	// Decades of the dimensionless conditioning number, so the distribution can be
	// read rather than guessed at. Widening or narrowing the coplanar reject band
	// is only worth doing if tests actually land in the decades it would move.
	inline constexpr int32 ConditioningDecades = 20;

	struct TAUTROPE_CORE_API ConditioningReport
	{
		long long Tests = 0;
		long long NearCoplanar = 0;
		long long NearCoplanarAccepted = 0;
		long long WellConditionedAccepted = 0;

		// Det came out bitwise zero. No threshold reaches these: the method divides
		// by Det, so they are unsolvable rather than badly conditioned.
		long long ExactlyZero = 0;

		// Of the near-coplanar tests, how many were against an edge lying flat
		// across a face -- the edges a rope should never have been offered.
		long long NearCoplanarOnInFaceEdge = 0;

		// Decade[k] counts tests with conditioning in [1e-(k+1), 1e-k); the last
		// bucket collects everything smaller but still nonzero.
		long long Decade[ConditioningDecades] = {};
	};

	// Re-runs every sweep/edge pair the collision phase would have tested, using
	// the recorded before and after positions, and reports how many were
	// degenerate. A high near-coplanar count with zero accepted means the rope is
	// sliding flat across edges that are never registering.
	TAUTROPE_CORE_API ConditioningReport AnalyseConditioning(const Recording& InRecording, double CoplanarThreshold = 1.0e-6);

	// Sweeps where more than one edge intersects the same sweep triangle at the
	// same barycentric point. Geometrically that point is a shape vertex: two
	// edges meeting there both cross the triangle at the corner they share. The
	// collision phase keeps a single best hit per sweep, so one of the two is
	// discarded, and which one survives decides whether the rope attaches to the
	// real silhouette edge or to the triangulation diagonal lying flat in a face.
	//
	// This is the benchmark for the tie fix: TiesAtSharedVertex is what needs
	// handling, and PointsBornOnInFaceEdge in EdgeUsageReport is what should stop
	// varying run to run once it is handled.
	struct TAUTROPE_CORE_API TiedSweepReport
	{
		long long Sweeps = 0;
		long long SweepsWithHit = 0;
		long long SweepsWithTie = 0;
		long long TiesInvolvingInFaceEdge = 0;
		long long TiesAtSharedVertex = 0;
		long long TiesAcrossShapes = 0;

		// Ties are only part of it. HitData holds ONE hit, so whenever a sweep
		// crosses more than one edge every loser is discarded, tie or not.
		long long SweepsWithMultipleHits = 0;
		long long InFaceEdgeWon = 0;
		long long InFaceEdgeLost = 0;
		long long InFaceEdgeAlsoReported = 0;
		long long GapExactlyZero = 0;
		long long GapUnderMilli = 0;
		long long GapOverMilli = 0;
		long long TiesAtSameLocation = 0;

		int32 FirstTieFrame = IndexNone;
		int32 FirstTieShape = IndexNone;
		int32 FirstTieEdgeA = IndexNone;
		int32 FirstTieEdgeB = IndexNone;
		int32 FirstTieSharedVert = IndexNone;
	};

	TAUTROPE_CORE_API TiedSweepReport AnalyseTiedSweeps(const Recording& InRecording, double RatioTolerance = 0.0);
}
