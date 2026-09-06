#pragma once

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/Core.h"
#include "TautRopeCore/Math.h"

#include <vector>

namespace TautRope
{
	class IDebugDraw;
	struct Point;

	struct HitData
	{
		bool bIsHit = false;
		bool bIsHitOnFirstTriangleSweep = false;
		int32 RopePointIndex = IndexNone;
		Vec3 Location;
		Vec3 OnSweepEdgeLocation;
		int32 ShapeIndex = IndexNone;
		int32 EdgeIndex = IndexNone;
		float SweepRatio = MaxFloat;
	};

	TAUTROPE_CORE_API void SweepRemovePoint(
		std::vector<Point>& RopePoints
		, const int32 RemovePointIndex
		, const std::vector<CollisionShape>& Shapes
		, IDebugDraw* Debug = nullptr
	);

	TAUTROPE_CORE_API void SweepSegmentThroughShapes(
		HitData& OutHitData
		, Point& InOutSegmentPointA
		, Point& InOutSegmentPointB
		, const Vec3& OriginLocationA
		, const Vec3& OriginLocationB
		, const Vec3& TargetLocationA
		, const Vec3& TargetLocationB
		, const std::vector<CollisionShape>& Shapes
		, const int32 RopePointIndex
		, IDebugDraw* Debug = nullptr
	);

	TAUTROPE_CORE_API void SweepSegmentTriangleAgainstShape(
		const Vec3& TriA
		, const Vec3& TriB
		, const Vec3& TriC
		, const CollisionShape& Shape
		, const int32 ShapeIndex
		, const int32 ShapeIndexPointA
		, const int32 ShapeIndexPointB
		, const int32 EdgeIndexPointA
		, const int32 EdgeIndexPointB
		, const int32 VertIndexPointA
		, const int32 VertIndexPointB
		, const int32 RopePointIndex
		, const bool bIsFirstTriangleSweep
		, HitData& OutHitData
	);

	TAUTROPE_CORE_API void SweepRemoveTriangleAgainstShape(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const CollisionShape& Shape
		, const int32 ShapeIndex
		, const std::vector<Int2>& IgnoredEdges
		, HitData& OutHitData
	);

	TAUTROPE_CORE_API bool GetTriangleLineIntersection(
		const Vec3& FromCorner
		, const Vec3& ToCorner
		, const Vec3& SupportCorner
		, const Vec3& LineA
		, const Vec3& LineB
		, Vec3& OutLocation
		, Vec3& OutOnSweepEdgeLocation
		, float& OutSweepRatio
	);

	TAUTROPE_CORE_API void DebugDrawSweep(
		IDebugDraw& Debug
		, const Vec3& A
		, const Vec3& B
		, const Vec3& C
		, bool bIsHit
	);
}
