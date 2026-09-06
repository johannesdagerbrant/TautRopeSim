#include "TautRopeDebugDraw.h"

#if TAUT_ROPE_DEBUG_DRAWING

#include "DrawDebugHelpers.h"
#include "Engine/World.h"

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

namespace
{
	FORCEINLINE FVector ToUE(const TautRope::Vec3& V)
	{
		return FVector(V.X, V.Y, V.Z);
	}

	FORCEINLINE FColor ToUE(unsigned int Color)
	{
		return FColor(
			static_cast<uint8>((Color >> 16) & 0xFFu)	// R
			, static_cast<uint8>((Color >> 8) & 0xFFu)	// G
			, static_cast<uint8>(Color & 0xFFu)			// B
			, static_cast<uint8>((Color >> 24) & 0xFFu)	// A
		);
	}
}

bool FTautRopeDebugDraw::IsUsable() const
{
	return IsValid(World);
}

void FTautRopeDebugDraw::Line(const TautRope::Vec3& A, const TautRope::Vec3& B, unsigned int Color)
{
	DrawDebugLine(World, ToUE(A), ToUE(B), ToUE(Color), false, -1.f, 0);
}

void FTautRopeDebugDraw::Sphere(const TautRope::Vec3& Center, double Radius, TautRope::int32 Segments, unsigned int Color)
{
	DrawDebugSphere(World, ToUE(Center), Radius, Segments, ToUE(Color));
}

void FTautRopeDebugDraw::Triangle(const TautRope::Vec3& A, const TautRope::Vec3& B, const TautRope::Vec3& C, unsigned int Color)
{
	const TArray<FVector> Vertices = { ToUE(A), ToUE(B), ToUE(C) };
	const TArray<int32> Indices = { 0, 1, 2 };
	DrawDebugMesh(World, Vertices, Indices, ToUE(Color), false, 5.f);
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

#endif // TAUT_ROPE_DEBUG_DRAWING
