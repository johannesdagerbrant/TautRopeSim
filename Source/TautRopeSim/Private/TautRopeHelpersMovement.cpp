
#include "TautRopeHelpersMovement.h"
#include "TautRopeConfig.h"

namespace TautRope
{
	FVector FindMinDistancePointBetweenABOnLineXY(
		const FVector& A
		, const FVector& B
		, const FVector& LineX
		, const FVector& LineY
		, float& OutDistAlongEdge
		, float& OutEdgeLength
	)
	{
		const FVector Edge = LineY - LineX;
		OutEdgeLength = Edge.Size();
		ensure(OutEdgeLength > KINDA_SMALL_NUMBER);

		const FVector EdgeDir = Edge / OutEdgeLength;

		// Projections of A and B onto the edge axis
		const float Alpha = FVector::DotProduct(A - LineX, EdgeDir);
		const float Beta = FVector::DotProduct(B - LineX, EdgeDir);

		// Perpendicular offsets
		const float rhoA = FVector::CrossProduct(A - LineX, EdgeDir).Size();
		const float rhoB = FVector::CrossProduct(B - LineX, EdgeDir).Size();

		if (rhoA < KINDA_SMALL_NUMBER && rhoB < KINDA_SMALL_NUMBER)
		{
			// Both lie essentially on the line -> midpoint
			OutDistAlongEdge = 0.5f * (Alpha + Beta);
		}
		else if (rhoA < KINDA_SMALL_NUMBER)
		{
			OutDistAlongEdge = Alpha;
		}
		else if (rhoB < KINDA_SMALL_NUMBER)
		{
			OutDistAlongEdge = Beta;
		}
		else
		{
			// Weighted average
			const float r = rhoA / (rhoA + rhoB);
			OutDistAlongEdge = (1.0f - r) * Alpha + r * Beta;
		}

		return LineX + EdgeDir * OutDistAlongEdge;
	}
}