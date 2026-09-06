#pragma once

#include "CoreMinimal.h"
#include "TautRopeConfig.h"

#if TAUT_ROPE_DEBUG_DRAWING

#include "TautRopeCore/CollisionShape.h"
#include "TautRopeCore/DebugDraw.h"

class UWorld;

// Renders the primitives the core simulation emits. This is the only place that
// knows about UWorld, and it is where the debug CVars are read.
class TAUTROPE_API FTautRopeDebugDraw final : public TautRope::IDebugDraw
{
public:
	explicit FTautRopeDebugDraw(const UWorld* InWorld) : World(InWorld) {}

	virtual void Line(const TautRope::Vec3& A, const TautRope::Vec3& B, unsigned int Color) override;
	virtual void Sphere(const TautRope::Vec3& Center, double Radius, TautRope::int32 Segments, unsigned int Color) override;
	virtual void Triangle(const TautRope::Vec3& A, const TautRope::Vec3& B, const TautRope::Vec3& C, unsigned int Color) override;

	virtual bool WantsSegmentSweep() const override;
	virtual bool WantsRemoveSweep() const override;

	bool IsUsable() const;

	static bool WantsRope();
	static bool WantsRopeTouchedEdges();

private:
	const UWorld* World = nullptr;
};

// Draws a collision shape: edges, per-edge up vectors, and vertices coloured by
// whether they are corners.
TAUTROPE_API void DrawCollisionShape(TautRope::IDebugDraw& Debug, const TautRope::CollisionShape& Shape);

#endif // TAUT_ROPE_DEBUG_DRAWING
