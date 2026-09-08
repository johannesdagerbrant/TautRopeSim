// Simulation behaviour, run against a shape captured from the real level.
//
// The two TEST_PENDING cases at the bottom are the defects this whole system was
// built to chase. They assert the behaviour we want, not the behaviour we have,
// and are excluded from the pass/fail total so a red suite always means a
// regression. Promote them to TEST when they hold.
#include "Framework.h"
#include "Support.h"

#include "TautRopeCore/Compare.h"
#include "TautRopeCore/Recording.h"
#include "TautRopeCore/Rope.h"

#include <cmath>
#include <cstdio>

using TautRope::Vec3;

namespace
{
	constexpr float MaxLength = 1500.f;

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

	struct SweepResult
	{
		double DeepestPenetration = 0.0;
		int MaxPointCount = 0;
		std::vector<TautRope::Point> FinalPoints;
	};

	SweepResult SweepAroundShape(const int FrameCount)
	{
		const TautRope::CollisionShape Shape = TautRopeTest::MakeCapturedBoxShape();
		const Vec3 Centre = ShapeCentre(Shape);

		TautRope::Rope Rope;
		Rope.AppendToNearbyShapes({ Shape });

		SweepResult Result;
		for (int Frame = 0; Frame < FrameCount; ++Frame)
		{
			Rope.UpdateRope(StartLocation, SweptEnd(Centre, Frame, FrameCount), MaxLength);

			const double Depth = TautRopeTest::DeepestPenetration(Rope.GetPoints(), Shape);
			if (Depth > Result.DeepestPenetration) { Result.DeepestPenetration = Depth; }

			const int Count = static_cast<int>(Rope.GetPoints().size());
			if (Count > Result.MaxPointCount) { Result.MaxPointCount = Count; }
		}
		Result.FinalPoints = Rope.GetPoints();
		return Result;
	}
}

TEST(Simulation_SeedsTwoPointsOnFirstUpdate)
{
	TautRope::Rope Rope;
	Rope.UpdateRope(Vec3(0.0, 0.0, 0.0), Vec3(100.0, 0.0, 0.0), MaxLength);
	CHECK_EQ(Rope.GetPoints().size(), std::size_t(2));
	CHECK_EQ(Rope.GetPoints()[0].Id, 0);
	CHECK_EQ(Rope.GetPoints()[1].Id, 1);
}

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

// --------------------------------------------------------------------------
// Known defects. See the note at the top of this file.
// --------------------------------------------------------------------------

TEST_PENDING(Defect_RopeNeverPenetratesShape)
{
	// "the rope also at times will intersect shapes after it has slid over a
	// vertex". A rope point should never end up inside a shape. Tolerance is one
	// unit, comfortably above the simulation's own 0.01 distance tolerance, so
	// only real penetration trips it.
	const SweepResult Result = SweepAroundShape(360);
	CHECK(Result.DeepestPenetration < 1.0);
	std::printf("      deepest penetration over the sweep: %.6f units\n", Result.DeepestPenetration);
}

TEST_PENDING(Defect_RopeSettlesWhenInputsStopMoving)
{
	// "the rope slows down as two or more points converge on edges towards the
	// same vertex". Held still, the rope should reach a resting configuration and
	// stop changing. If points converging on a vertex crawl instead of resolving,
	// the length keeps creeping long after the endpoints stopped.
	const TautRope::CollisionShape Shape = TautRopeTest::MakeCapturedBoxShape();
	const Vec3 Centre = ShapeCentre(Shape);

	TautRope::Rope Rope;
	Rope.AppendToNearbyShapes({ Shape });

	// Wrap the shape first, then hold everything still.
	constexpr int SweepFrames = 200;
	for (int Frame = 0; Frame < SweepFrames; ++Frame)
	{
		Rope.UpdateRope(StartLocation, SweptEnd(Centre, Frame, SweepFrames * 2), MaxLength);
	}

	const Vec3 HeldEnd = SweptEnd(Centre, SweepFrames, SweepFrames * 2);
	for (int Frame = 0; Frame < 200; ++Frame)
	{
		Rope.UpdateRope(StartLocation, HeldEnd, MaxLength);
	}

	const double LengthBefore = TautRopeTest::RopeLength(Rope.GetPoints());
	const std::size_t CountBefore = Rope.GetPoints().size();

	for (int Frame = 0; Frame < 100; ++Frame)
	{
		Rope.UpdateRope(StartLocation, HeldEnd, MaxLength);
	}

	const double LengthAfter = TautRopeTest::RopeLength(Rope.GetPoints());
	const double Drift = std::fabs(LengthAfter - LengthBefore);

	std::printf("      length drift over 100 static frames: %.9f units (%zu -> %zu points)\n",
		Drift, CountBefore, Rope.GetPoints().size());
	CHECK(Drift < 0.01);
}
