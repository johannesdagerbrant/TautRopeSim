// The math types were ported from FVector/FQuat/FMath, and the port had to
// preserve their exact semantics -- including the odd corners -- because a
// recording captured before the port has to keep replaying afterwards.
#include "Framework.h"

#include "TautRopeCore/Math.h"

using TautRope::Quat;
using TautRope::Vec3;

// PROVES: a vector already of unit length comes back bit-identical.
// GUARDS: the SquareSum == 1.0 early-out, which the engine has and which keeps
// an already-normalised vector from being multiplied by a rounded 1.0.
// NOT YET SABOTAGE-PROVEN: removing the early-out leaves this green, because for
// a true unit vector both paths are exact. It documents intent, not a guard.
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

// PROVES: a vector shorter than the tolerance normalises to exactly zero rather
// than to garbage from dividing by a near-zero length.
// GUARDS: the tolerance branch. Sabotage: change the guard to SquareSum < 0.0 and
// this goes red.
TEST(GetSafeNormal_ReturnsZeroBelowTolerance)
{
	const Vec3 Tiny(1.0e-12, 0.0, 0.0);
	const Vec3 Normalised = Tiny.GetSafeNormal();
	CHECK_BITS(Normalised.X, 0.0);
	CHECK_BITS(Normalised.Y, 0.0);
	CHECK_BITS(Normalised.Z, 0.0);
}

// PROVES: normalisation is X * (1/sqrt(sum)), not X / sqrt(sum). The two differ
// in the last bit, and the engine does the former.
// GUARDS: bit-identity with UE, which the whole replay loop rests on. Sabotage:
// divide each component by sqrt(SquareSum) and this goes red.
TEST(GetSafeNormal_UsesReciprocalSquareRoot)
{
	// Must be X * (1 / sqrt(sum)), not X / sqrt(sum): the two differ in the last
	// bit and the engine does the former.
	//
	// The vector matters. This test used V(3,4,12), a Pythagorean quadruple whose
	// SizeSquared is 169 and whose square root is exactly 13, so every ordering of
	// the arithmetic agrees to the bit and the test passed under deliberate
	// sabotage. Use a sum whose root is irrational.
	const Vec3 V(0.3, 1.7, 2.9);
	const double SquareSum = V.SizeSquared();
	const double Scale = 1.0 / std::sqrt(SquareSum);
	const Vec3 Normalised = V.GetSafeNormal();
	CHECK_BITS(Normalised.X, V.X * Scale);
	CHECK_BITS(Normalised.Y, V.Y * Scale);
	CHECK_BITS(Normalised.Z, V.Z * Scale);
}

// PROVES: Clamp resolves in FMath's branch ORDER, not merely to the same answer.
// GUARDS: bit-identity with UE. Sabotage: reverse the comparisons and this goes
// red on the inverted-range and NaN cases, which are the only inputs that can
// tell the two orderings apart.
TEST(Clamp_MatchesEngineComparisonOrder)
{
	CHECK_EQ(TautRope::Math::Clamp(5.0, 0.0, 10.0), 5.0);
	CHECK_EQ(TautRope::Math::Clamp(-1.0, 0.0, 10.0), 0.0);
	CHECK_EQ(TautRope::Math::Clamp(11.0, 0.0, 10.0), 10.0);
	// Exactly at the upper bound: FMath::Clamp returns Max via its final branch.
	CHECK_EQ(TautRope::Math::Clamp(10.0, 0.0, 10.0), 10.0);
	CHECK_EQ(TautRope::Math::Clamp(0.0, 0.0, 10.0), 0.0);

	// The branch ORDER, which is the thing this test is named for, is invisible on
	// ordinary inputs: reversing the comparisons gives identical answers for every
	// case above, and the test passed under exactly that sabotage. Two inputs
	// distinguish them. An inverted range resolves through Min, and NaN fails every
	// comparison so it falls out of the last branch.
	CHECK_EQ(TautRope::Math::Clamp(5.0, 10.0, 0.0), 10.0);
	const double NotANumber = std::numeric_limits<double>::quiet_NaN();
	CHECK_EQ(TautRope::Math::Clamp(NotANumber, 0.0, 10.0), 10.0);
}

// PROVES: the identity quaternion yields UE's axis convention, X forward, Z up.
// GUARDS: the handedness of every edge rotation read from a recording. A mirrored
// convention would put rope points on the wrong side of every edge.
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

// PROVES: RotateVector evaluates in the engine's operation order, V + W*T +
// Cross(Q,T) with T = 2*Cross(Q,V).
// GUARDS: bit-identity with UE. Any algebraically equivalent regrouping rounds
// differently and breaks replay verification.
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

// PROVES: Lerp is A + Alpha*(B-A), not the algebraically equal A*(1-Alpha)+B*Alpha.
// GUARDS: bit-identity with UE. Sabotage: substitute the other form and this goes
// red, which is exactly the point -- the two agree mathematically and differ in
// floating point.
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

// PROVES: Dist and SizeSquared agree, so rope length and the sweep tests measure
// the same geometry.
// GUARDS: an inconsistent distance pair, which would make length drift and
// collision disagree about whether two points are touching.
TEST(Dist_IsSquareRootOfSquaredDistance)
{
	const Vec3 A(1.0, 2.0, 3.0);
	const Vec3 B(4.0, 6.0, 15.0);
	CHECK_BITS(Vec3::Dist(A, B), (B - A).Size());
}
