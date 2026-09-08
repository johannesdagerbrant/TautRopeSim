// The math types were ported from FVector/FQuat/FMath, and the port had to
// preserve their exact semantics -- including the odd corners -- because a
// recording captured before the port has to keep replaying afterwards.
#include "Framework.h"

#include "TautRopeCore/Math.h"

using TautRope::Quat;
using TautRope::Vec3;

TEST(GetSafeNormal_ReturnsInputUnchangedWhenAlreadyUnitLength)
{
	// FVector::GetSafeNormal short-circuits on SquareSum == 1 and returns the
	// input untouched rather than dividing by one. Dropping that early-out would
	// change results in the last bits.
	const Vec3 UnitX(1.0, 0.0, 0.0);
	const Vec3 Normalised = UnitX.GetSafeNormal();
	CHECK_BITS(Normalised.X, UnitX.X);
	CHECK_BITS(Normalised.Y, UnitX.Y);
	CHECK_BITS(Normalised.Z, UnitX.Z);
}

TEST(GetSafeNormal_ReturnsZeroBelowTolerance)
{
	const Vec3 Tiny(1.0e-12, 0.0, 0.0);
	const Vec3 Normalised = Tiny.GetSafeNormal();
	CHECK_BITS(Normalised.X, 0.0);
	CHECK_BITS(Normalised.Y, 0.0);
	CHECK_BITS(Normalised.Z, 0.0);
}

TEST(GetSafeNormal_UsesReciprocalSquareRoot)
{
	// Must be X * (1 / sqrt(sum)), not X / sqrt(sum): the two differ in the last
	// bit and the engine does the former.
	const Vec3 V(3.0, 4.0, 12.0);
	const double SquareSum = V.SizeSquared();
	const double Scale = 1.0 / std::sqrt(SquareSum);
	const Vec3 Normalised = V.GetSafeNormal();
	CHECK_BITS(Normalised.X, V.X * Scale);
	CHECK_BITS(Normalised.Y, V.Y * Scale);
	CHECK_BITS(Normalised.Z, V.Z * Scale);
}

TEST(Clamp_MatchesEngineComparisonOrder)
{
	CHECK_EQ(TautRope::Math::Clamp(5.0, 0.0, 10.0), 5.0);
	CHECK_EQ(TautRope::Math::Clamp(-1.0, 0.0, 10.0), 0.0);
	CHECK_EQ(TautRope::Math::Clamp(11.0, 0.0, 10.0), 10.0);
	// Exactly at the upper bound: FMath::Clamp returns Max via its final branch.
	CHECK_EQ(TautRope::Math::Clamp(10.0, 0.0, 10.0), 10.0);
	CHECK_EQ(TautRope::Math::Clamp(0.0, 0.0, 10.0), 0.0);
}

TEST(Quat_IdentityAxesMatchEngineConvention)
{
	const Quat Identity;
	const Vec3 Forward = Identity.GetForwardVector();
	const Vec3 Up = Identity.GetUpVector();
	CHECK_BITS(Forward.X, 1.0);
	CHECK_BITS(Forward.Y, 0.0);
	CHECK_BITS(Forward.Z, 0.0);
	CHECK_BITS(Up.X, 0.0);
	CHECK_BITS(Up.Y, 0.0);
	CHECK_BITS(Up.Z, 1.0);
}

TEST(Quat_RotateVectorFollowsEngineOperationOrder)
{
	// FQuat::RotateVector computes V + (W * T) + Cross(Q, T) with T = 2 * Cross(Q, V).
	// Any algebraically equivalent regrouping changes the last bits.
	const Quat Q(0.1, 0.2, 0.3, 0.9);
	const Vec3 V(1.5, -2.5, 3.5);

	const Vec3 QVec(Q.X, Q.Y, Q.Z);
	const Vec3 T = 2.0 * Vec3::Cross(QVec, V);
	const Vec3 Expected = V + (Q.W * T) + Vec3::Cross(QVec, T);

	const Vec3 Actual = Q.RotateVector(V);
	CHECK_BITS(Actual.X, Expected.X);
	CHECK_BITS(Actual.Y, Expected.Y);
	CHECK_BITS(Actual.Z, Expected.Z);
}

TEST(Lerp_MatchesEngineFormula)
{
	// FMath::Lerp is A + Alpha * (B - A), which is not bitwise equal to the
	// (1 - Alpha) * A + Alpha * B form.
	const Vec3 A(1.0, 2.0, 3.0);
	const Vec3 B(-7.0, 11.0, 0.25);
	const double Alpha = 1.0 / 3.0;

	const Vec3 Expected = A + Alpha * (B - A);
	const Vec3 Actual = TautRope::Math::Lerp(A, B, Alpha);
	CHECK_BITS(Actual.X, Expected.X);
	CHECK_BITS(Actual.Y, Expected.Y);
	CHECK_BITS(Actual.Z, Expected.Z);
}

TEST(Dist_IsSquareRootOfSquaredDistance)
{
	const Vec3 A(1.0, 2.0, 3.0);
	const Vec3 B(4.0, 6.0, 15.0);
	CHECK_BITS(Vec3::Dist(A, B), (B - A).Size());
}
