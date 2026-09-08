#include "TautRopeDebugDraw.h"

#if TAUT_ROPE_DEBUG_DRAWING

#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "TautRopeConvert.h"

static TAutoConsoleVariable<int32> CVarDrawDebugRope(
	TEXT("TautRope.DrawDebugRope"),
	0,
	TEXT("Draw rope debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugRopeTouchedEdges(
	TEXT("TautRope.DrawDebugRopeTouchedEdges"),
	0,
	TEXT("Draw rope touched shape edges debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugSegmentSweep(
	TEXT("TautRope.DrawDebugSegmentSweep"),
	0,
	TEXT("Draw rope segment sweep debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

static TAutoConsoleVariable<int32> CVarDrawDebugRemoveSweep(
	TEXT("TautRope.DrawDebugRemoveSweep"),
	0,
	TEXT("Draw rope point removal sweep debug visualization.\n")
	TEXT("0: Off\n")
	TEXT("1: On"),
	ECVF_Cheat
);

using TautRopeConvert::ToColor;
using TautRopeConvert::ToUE;

bool FTautRopeDebugDraw::IsUsable() const
{
	return IsValid(World);
}

void FTautRopeDebugDraw::Line(const TautRope::Vec3& A, const TautRope::Vec3& B, unsigned int Color)
{
	DrawDebugLine(World, ToUE(A), ToUE(B), ToColor(Color), false, -1.f, 0);
}

void FTautRopeDebugDraw::Sphere(const TautRope::Vec3& Center, double Radius, TautRope::int32 Segments, unsigned int Color)
{
	DrawDebugSphere(World, ToUE(Center), Radius, Segments, ToColor(Color));
}

void FTautRopeDebugDraw::Triangle(const TautRope::Vec3& A, const TautRope::Vec3& B, const TautRope::Vec3& C, unsigned int Color)
{
	const TArray<FVector> Vertices = { ToUE(A), ToUE(B), ToUE(C) };
	const TArray<int32> Indices = { 0, 1, 2 };
	DrawDebugMesh(World, Vertices, Indices, ToColor(Color), false, 5.f);
}

bool FTautRopeDebugDraw::WantsSegmentSweep() const
{
	return CVarDrawDebugSegmentSweep.GetValueOnGameThread() != 0;
}

bool FTautRopeDebugDraw::WantsRemoveSweep() const
{
	return CVarDrawDebugRemoveSweep.GetValueOnGameThread() != 0;
}

bool FTautRopeDebugDraw::WantsRope()
{
	return CVarDrawDebugRope.GetValueOnGameThread() != 0;
}

bool FTautRopeDebugDraw::WantsRopeTouchedEdges()
{
	return CVarDrawDebugRopeTouchedEdges.GetValueOnGameThread() != 0;
}

void DrawCollisionShape(TautRope::IDebugDraw& Debug, const TautRope::CollisionShape& Shape)
{
	for (int32 EdgeIndex = 0; EdgeIndex < static_cast<int32>(Shape.Edges.size()); ++EdgeIndex)
	{
		const TautRope::Int2& Edge = Shape.Edges[EdgeIndex];
		const TautRope::Vec3& EdgeVertA = Shape.Vertices[Edge.X];
		const TautRope::Vec3& EdgeVertB = Shape.Vertices[Edge.Y];
		Debug.Line(EdgeVertA, EdgeVertB, TautRope::ColorBlue);

		const TautRope::Vec3 Center = (EdgeVertA + EdgeVertB) * 0.5;
		const double Length = TautRope::Vec3::Dist(EdgeVertA, EdgeVertB) * 0.05;
		Debug.Line(Center, Center + Shape.EdgeRotations[EdgeIndex].GetUpVector() * Length, TautRope::ColorYellow);
	}
	for (int32 VertIndex = 0; VertIndex < static_cast<int32>(Shape.Vertices.size()); ++VertIndex)
	{
		Debug.Sphere(
			Shape.Vertices[VertIndex]
			, 0.5
			, 4
			, Shape.IsCornerVertex(VertIndex) ? TautRope::ColorRed : TautRope::ColorYellow
		);
	}
}

#endif // TAUT_ROPE_DEBUG_DRAWING
