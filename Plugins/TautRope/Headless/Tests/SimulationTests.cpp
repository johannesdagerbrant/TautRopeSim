// Simulation behaviour, run against a shape captured from the real level.
//
// Every test here states what it PROVES, then what it FIXES if it closed a
// debugging loop, or what it GUARDS otherwise, and the sabotage that was seen to
// turn it red. A test nobody has watched fail asserts nothing, and reading it
// cannot tell you which kind it is.
#include "Framework.h"
#include "Support.h"

#include "TautRopeCore/Compare.h"
#include "TautRopeCore/Config.h"
#include "TautRopeCore/Recording.h"
#include "TautRopeCore/Rope.h"

#include <cmath>
#include <cstdio>
#include <cstddef>

using TautRope::Vec3;

namespace
{
	constexpr float MaxLength = 1500.f;

	enum class SweepMode { OneWay, ThereAndBack, OverTheTop };

	// Outside the shape on the low-Y side, level with the middle of the box.
	const Vec3 StartLocation(-463.0, 100.0, 162.0);

	Vec3 ShapeCentre(const TautRope::CollisionShape& Shape)
	{
		const TautRopeTest::Bounds B = TautRopeTest::ShapeBounds(Shape);
		return Vec3(
			(B.Min.X + B.Max.X) * 0.5
			, (B.Min.Y + B.Max.Y) * 0.5
			, (B.Min.Z + B.Max.Z) * 0.5
		);
	}

	// Sweeps the free end around the shape, which is what forces the rope to wrap
	// edges and slide across vertices.
	Vec3 SweptEnd(const Vec3& Centre, const int Frame, const int FrameCount)
	{
		const double Angle = (6.283185307179586 * Frame) / FrameCount;
		const double Radius = 260.0;
		return Vec3(
			Centre.X + Radius * std::cos(Angle)
			, Centre.Y + Radius * std::sin(Angle)
			, StartLocation.Z
		);
	}

	// Around the shape and back again. The return leg is the point: sweeping one
	// way only ever adds rope points, and the pruning phase removes a converged
	// group when the rope unwraps. No unwrap, no slide over a vertex, and both
	// defect tests below assert against a rope that never did the thing.
	Vec3 SweptEndThereAndBack(const Vec3& Centre, const int Frame, const int FrameCount)
	{
		const int Half = FrameCount / 2;
		const int Effective = Frame <= Half ? Frame : FrameCount - Frame;
		return SweptEnd(Centre, Effective, Half);
	}

	// Over the top of the shape rather than around its waist. The reported defect
	// happens where the rope wraps a top corner: points converge on the vertex two
	// silhouette edges and a face diagonal share. A sweep in the horizontal plane
	// never puts the rope on a top face at all.
	Vec3 SweptEndOverTheTop(const Vec3& Centre, const double TopZ, const int Frame, const int FrameCount)
	{
		const int Half = FrameCount / 2;
		const int Effective = Frame <= Half ? Frame : FrameCount - Frame;
		const double Angle = (3.141592653589793 * Effective) / Half;
		const double Radius = 260.0;
		// Starts on the same side as StartLocation, so the rope begins as a short
		// straight line clear of the shape, then is dragged up over the top face and
		// down the far side. Starting on the far side puts the very first straight
		// line through the solid, which is a broken fixture rather than a defect:
		// that arrangement reported 105 units inside on frame 0.
		return Vec3(
			Centre.X
			, Centre.Y - Radius * std::cos(Angle)
			, TopZ + Radius * std::sin(Angle)
		);
	}

	struct SweepResult
	{
		double DeepestPenetration = 0.0;
		double DeepestSegmentInside = 0.0;
		double FirstFrameSegmentInside = 0.0;
		int FirstPenetratingFrame = -1;
		int MaxPointCount = 0;
		int MinPointCount = 1 << 30;
		int RemovalFrames = 0;
		int PointsRemoved = 0;
		int CollisionIterationCapHits = 0;
		int MostCollisionIterations = 0;
		int RemoveSweepIterationCapHits = 0;
		int MostRemoveSweepIterations = 0;
		std::vector<TautRope::Point> FinalPoints;
	};

	SweepResult RunSweep(const int FrameCount, const SweepMode Mode)
	{
		const TautRope::CollisionShape Shape = TautRopeTest::MakeCapturedBoxShape();
		const Vec3 Centre = ShapeCentre(Shape);
		const double TopZ = TautRopeTest::ShapeBounds(Shape).Max.Z;

		TautRope::Rope Rope;
		Rope.AppendToNearbyShapes({ Shape });

		SweepResult Result;
		int PreviousCount = 0;
		for (int Frame = 0; Frame < FrameCount; ++Frame)
		{
			const Vec3 End = Mode == SweepMode::OverTheTop
				? SweptEndOverTheTop(Centre, TopZ, Frame, FrameCount)
				: (Mode == SweepMode::ThereAndBack
					? SweptEndThereAndBack(Centre, Frame, FrameCount)
					: SweptEnd(Centre, Frame, FrameCount));
			Rope.UpdateRope(StartLocation, End, MaxLength);

			const double Depth = TautRopeTest::DeepestPenetration(Rope.GetPoints(), Shape);
			if (Depth > Result.DeepestPenetration) { Result.DeepestPenetration = Depth; }

			const double Inside = TautRopeTest::DeepestSegmentInside(Rope.GetPoints(), Shape);
			if (Inside > Result.DeepestSegmentInside) { Result.DeepestSegmentInside = Inside; }
			if (Frame == 0) { Result.FirstFrameSegmentInside = Inside; }
			if (Inside > 1.0 && Result.FirstPenetratingFrame < 0) { Result.FirstPenetratingFrame = Frame; }

			const int Count = static_cast<int>(Rope.GetPoints().size());
			if (Count > Result.MaxPointCount) { Result.MaxPointCount = Count; }
			if (Count < Result.MinPointCount) { Result.MinPointCount = Count; }

			// A drop in the point count is the pruning phase removing a converged
			// group, i.e. the rope sliding over a vertex. That is the event both
			// defects need, so the fixture has to be seen producing it.
			if (PreviousCount > 0 && Count < PreviousCount)
			{
				++Result.RemovalFrames;
				Result.PointsRemoved += PreviousCount - Count;
			}
			PreviousCount = Count;
		}
		Result.FinalPoints = Rope.GetPoints();

		Result.CollisionIterationCapHits = Rope.CollisionIterationCapHits;
		Result.MostCollisionIterations = Rope.MostCollisionIterations;
		Result.RemoveSweepIterationCapHits = Rope.RemoveSweepIterationCapHits;
		Result.MostRemoveSweepIterations = Rope.MostRemoveSweepIterations;
		return Result;
	}

	SweepResult SweepAroundShape(const int FrameCount)
	{
		return RunSweep(FrameCount, SweepMode::OneWay);
	}

	SweepResult SweepAroundShapeAndBack(const int FrameCount)
	{
		return RunSweep(FrameCount, SweepMode::ThereAndBack);
	}

	SweepResult SweepOverTheTop(const int FrameCount)
	{
		return RunSweep(FrameCount, SweepMode::OverTheTop);
	}
}

// PROVES: the first update seeds exactly the two endpoints.
// GUARDS: the starting state every other simulation test builds on.
TEST(Simulation_SeedsTwoPointsOnFirstUpdate)
{
	TautRope::Rope Rope;
	Rope.UpdateRope(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0), MaxLength);
	CHECK_EQ(Rope.GetPoints().size(), std::size_t(2));
	CHECK_EQ(Rope.GetPoints()[0].Id, 0);
	CHECK_EQ(Rope.GetPoints()[1].Id, 1);
}

// PROVES: the same inputs replayed twice produce bit-identical output.
// GUARDS: the premise of the entire headless loop. If this fails, no measurement
// taken from a replay says anything about the editor.
// NOT YET SABOTAGE-PROVEN: perturbing an input changes both runs equally, so they
// still agree. Only genuine run-to-run variance, which nothing here introduces,
// would redden it.
TEST(Simulation_IsDeterministic)
{
	// Two ropes, identical inputs, compared bitwise through the same routine the
	// replay verification uses. If this fails, nothing downstream can be trusted.
	const TautRope::CollisionShape Shape = TautRopeTest::MakeCapturedBoxShape();
	const Vec3 Centre = ShapeCentre(Shape);
	constexpr int FrameCount = 120;

	TautRope::Recording RunA;
	TautRope::Recording RunB;

	for (int Pass = 0; Pass < 2; ++Pass)
	{
		TautRope::Rope Rope;
		Rope.AppendToNearbyShapes({ Shape });
		TautRope::Recording& Out = (Pass == 0) ? RunA : RunB;

		for (int Frame = 0; Frame < FrameCount; ++Frame)
		{
			TautRope::RecordedFrame Recorded;
			Recorded.StartLocation = StartLocation;
			Recorded.EndLocation = SweptEnd(Centre, Frame, FrameCount);
			Recorded.MaxLength = MaxLength;
			Rope.UpdateRope(Recorded.StartLocation, Recorded.EndLocation, Recorded.MaxLength, nullptr, &Recorded.Capture);
			Out.Frames.push_back(Recorded);
		}
	}

	TautRope::RecordingDivergence Divergence;
	const bool bIdentical = TautRope::CompareRecordings(RunA, RunB, Divergence);
	CHECK(bIdentical);
	if (!bIdentical)
	{
		std::printf("      diverged frame %d, %s / %s\n",
			Divergence.FrameIndex, Divergence.Phase, Divergence.Field);
	}
}

// PROVES: the sweep actually makes the rope gain intermediate points.
// GUARDS: every test below from asserting against a straight line that never
// touched the shape.
TEST(Simulation_WrapsShapeWhenEndIsSweptAround)
{
	// If the rope never gains an intermediate point, the collision and pruning
	// phases are not being exercised at all and the tests below prove nothing.
	const SweepResult Result = SweepAroundShape(360);
	CHECK(Result.MaxPointCount > 2);
	if (Result.MaxPointCount <= 2)
	{
		std::printf("      rope stayed a straight line; the fixture is not being wrapped\n");
	}
}


// PROVES: neither the collision loop nor the remove sweep ever reaches its
// ceiling, and the rope never reaches the point ceiling.
// GUARDS: runaway insertion. The ceilings exist only to stop point counts
// exploding until the simulation freezes and runs out of memory, so touching one
// is a bug and never a heavy frame. Sabotage: make the straightening step
// overshoot its target and this goes red with 13 collision and 322 remove-sweep
// frames at the cap.
TEST(Simulation_NeverExhaustsCollisionIterations)
{
	const SweepResult Result = SweepAroundShape(360);
	CHECK_EQ(Result.CollisionIterationCapHits, 0);
	CHECK_EQ(Result.RemoveSweepIterationCapHits, 0);
	CHECK(Result.MostCollisionIterations < TautRope::MaxCollisionIterations);
	CHECK(Result.MostRemoveSweepIterations < TautRope::MaxRemoveSweepIterations);
	std::printf("      worst frame settled in %d of %d collision iterations,"
		" %d of %d remove-sweep rounds\n",
		Result.MostCollisionIterations, TautRope::MaxCollisionIterations,
		Result.MostRemoveSweepIterations, TautRope::MaxRemoveSweepIterations);
	if (Result.MostRemoveSweepIterations == 0)
	{
		// Said out loud because a ceiling nothing reaches is a ceiling nothing
		// tests. The recordings do exercise it (up to 3 rounds), this fixture
		// does not.
		std::printf("      note: the remove sweep never ran, so its ceiling is\n"
			"      untested by this fixture\n");
	}
	if (Result.CollisionIterationCapHits > 0 || Result.RemoveSweepIterationCapHits > 0)
	{
		std::printf("      %d collision and %d remove-sweep frame(s) hit the cap:\n"
			"      points were still being inserted when the loop gave up\n",
			Result.CollisionIterationCapHits, Result.RemoveSweepIterationCapHits);
	}
}

// PROVES: the rope never exceeds MaxLength.
// GUARDS: the length budget in the movement phase.
// NOT YET SABOTAGE-PROVEN: MaxLength is 1500 and this fixture never gets near it,
// so even letting the endpoint overshoot threefold leaves the test green. It
// needs a fixture that pulls the rope taut before it asserts anything.
TEST(Simulation_RopeStaysWithinMaxLength)
{
	const SweepResult Result = SweepAroundShape(360);
	const double Length = TautRopeTest::RopeLength(Result.FinalPoints);
	CHECK(Length <= static_cast<double>(MaxLength) + 1.0);
	if (Length > static_cast<double>(MaxLength) + 1.0)
	{
		std::printf("      rope length %.6f exceeds MaxLength %.1f\n", Length, MaxLength);
	}
}


// PROVES: no part of the rope ever lies inside a shape -- the LINE between two
// points, not just the points themselves.
// GUARDS: collision integrity. Sabotage: make GetTriangleLineIntersection return
// false and this goes red, clean on frame 0 then 113.707 units of rope through
// the box by frame 270.
//
// The measurement is segment-based on purpose. This test spent weeks green while
// checking POINT penetration, which the defect never produces: both endpoints
// rest on the surface and the segment between them cuts through. It also swept
// only around the shape's waist, never over a top face, so the rope was never on
// the geometry where the reported defect happens.
//
// It does NOT guard the tied-edge fix. Reverting that fix leaves this green on
// all three motions, so the synthetic fixture still does not reproduce the
// penetration seen in recording 192009. Until it does, that fix is covered only
// by the recordings.
TEST(Simulation_RopeNeverPassesThroughAShape)
{
	const SweepResult Around = SweepAroundShapeAndBack(720);
	const SweepResult OverTop = SweepOverTheTop(720);

	// A fixture that starts inside the shape proves nothing about sliding over
	// anything. The first arrangement of the over-the-top sweep began with the
	// straight rope through the box, reporting 105 units inside on frame 0.
	CHECK(Around.FirstFrameSegmentInside < 1.0);
	CHECK(OverTop.FirstFrameSegmentInside < 1.0);

	CHECK(Around.DeepestSegmentInside < 1.0);
	CHECK(OverTop.DeepestSegmentInside < 1.0);
	std::printf("      deepest rope line inside: %.6f around, %.6f over the top\n",
		Around.DeepestSegmentInside, OverTop.DeepestSegmentInside);
}

// PROVES: the fixture reaches the state both defects need -- the rope wraps, and
// the pruning phase later removes a converged group, which is the rope sliding
// over a vertex.
// GUARDS: the tests above from going quietly vacuous. Before the return leg was
// added the sweep only ever accumulated points, removed none, and every defect
// assertion held against a rope that had never slid over anything. Sabotage: drop
// the return leg from SweptEndThereAndBack and the removal count goes to zero.
TEST(Simulation_FixtureActuallySlidesOverAVertex)
{
	const SweepResult Around = SweepAroundShapeAndBack(720);
	const SweepResult OverTop = SweepOverTheTop(720);
	std::printf("      removals: %d frames / %d points around, %d frames / %d points over the top\n",
		Around.RemovalFrames, Around.PointsRemoved, OverTop.RemovalFrames, OverTop.PointsRemoved);
	CHECK(Around.MaxPointCount > 2);
	CHECK(Around.RemovalFrames > 0);
	CHECK(OverTop.RemovalFrames > 0);
}

// PROVES: held still, the rope reaches a resting configuration within a bounded
// number of frames.
// GUARDS: convergence progress. Sabotage: perturb an intermediate target by a
// constant each frame in MovementPhase and the rope never settles, so this goes
// red at the cap.
//
// It measures frames-to-settle from the moment the input stops, not drift after
// a warmup. The previous version held 200 frames and only then measured 100 more,
// so it could only ever observe an already-settled rope -- no sabotage reached
// it, including breaking the iteration cap and letting the movement phase
// overshoot.
TEST(Simulation_RopeSettlesWhenInputsStopMoving)
{
	const TautRope::CollisionShape Shape = TautRopeTest::MakeCapturedBoxShape();
	const Vec3 Centre = ShapeCentre(Shape);

	TautRope::Rope Rope;
	Rope.AppendToNearbyShapes({ Shape });

	constexpr int SweepFrames = 200;
	for (int Frame = 0; Frame < SweepFrames; ++Frame)
	{
		Rope.UpdateRope(StartLocation, SweptEnd(Centre, Frame, SweepFrames * 2), MaxLength);
	}

	const Vec3 HeldEnd = SweptEnd(Centre, SweepFrames, SweepFrames * 2);
	constexpr int SettleCap = 240;
	// A rope that is not settling usually grows points as well, and the iteration
	// ceilings bound work within a frame, not growth across frames. Without this
	// the test does not fail, it grinds: one sabotage run took over nine minutes
	// without finishing 240 frames. Fail fast instead.
	constexpr std::size_t PointCeiling = 64;
	std::size_t WorstPointCount = Rope.GetPoints().size();
	double Previous = TautRopeTest::RopeLength(Rope.GetPoints());
	int FramesToSettle = -1;
	int Quiet = 0;
	for (int Frame = 0; Frame < SettleCap; ++Frame)
	{
		Rope.UpdateRope(StartLocation, HeldEnd, MaxLength);
		WorstPointCount = Rope.GetPoints().size() > WorstPointCount ? Rope.GetPoints().size() : WorstPointCount;
		if (WorstPointCount > PointCeiling)
		{
			break;
		}
		const double Length = TautRopeTest::RopeLength(Rope.GetPoints());
		// Ten consecutive frames that move the rope less than a hundredth of the
		// simulation's own distance tolerance count as settled.
		Quiet = std::fabs(Length - Previous) < 1.0e-4 ? Quiet + 1 : 0;
		Previous = Length;
		if (Quiet >= 10)
		{
			FramesToSettle = Frame + 1;
			break;
		}
	}

	std::printf("      settled after %d frames (cap %d), %zu points, peak %zu\n",
		FramesToSettle, SettleCap, Rope.GetPoints().size(), WorstPointCount);
	CHECK(WorstPointCount <= PointCeiling);
	CHECK(FramesToSettle > 0);
}
