// The recording format is the boundary the whole debugging loop rests on: if a
// double does not survive the round trip exactly, bit-identity is lost at the
// file rather than in the simulation, and every result downstream is suspect.
#include "Framework.h"
#include "Support.h"

#include "TautRopeCore/Compare.h"
#include "TautRopeCore/Recording.h"

#include <cstdio>
#include <limits>

using TautRope::Int2;
using TautRope::Quat;
using TautRope::Vec3;

namespace
{
	// Values chosen to break a naive writer: %f loses them, %.15g loses some,
	// only round-trip precision keeps them all.
	const double Nasty[] = {
		1.0 / 3.0,
		3.14159265358979311599796346854,
		-0.0,
		1e-300,
		1e300,
		std::numeric_limits<double>::denorm_min(),
		std::numeric_limits<double>::min(),
		std::numeric_limits<double>::max(),
		0.1,
		123456789.123456789,
		-987654321.987654321,
	};
	constexpr int NastyCount = static_cast<int>(sizeof(Nasty) / sizeof(Nasty[0]));

	double NastyAt(int i) { return Nasty[((i % NastyCount) + NastyCount) % NastyCount]; }

	TautRope::Recording MakeAwkwardRecording()
	{
		TautRope::Recording R;
		R.EngineBuild = "test-build";
		R.NextPointId = 4242;

		TautRope::CollisionShape Shape;
		for (int i = 0; i < 6; ++i)
		{
			Shape.Vertices.push_back(Vec3(NastyAt(i), NastyAt(i + 1), NastyAt(i + 2)));
		}
		for (int i = 0; i < 9; ++i)
		{
			Shape.Edges.push_back(Int2(i % 6, (i + 1) % 6));
			Shape.EdgeRotations.push_back(Quat(NastyAt(i), NastyAt(i + 3), NastyAt(i + 5), NastyAt(i + 7)));
		}
		for (int i = 0; i < 6; ++i)
		{
			Shape.VertToEdges.push_back({ i, (i + 1) % 9 });
			Shape.IsCornerVertexList.push_back((i % 2) == 0);
		}
		R.Shapes.push_back(Shape);
		R.Shapes.push_back(Shape);

		for (int i = 0; i < 3; ++i)
		{
			TautRope::RecordedPoint P;
			P.Id = i;
			P.Location = Vec3(NastyAt(i), NastyAt(i + 4), NastyAt(i + 8));
			P.ShapeIndex = i % 2;
			P.EdgeIndex = i;
			P.VertIndex = (i == 1) ? TautRope::IndexNone : i;
			R.InitialPoints.push_back(P);
		}

		for (int f = 0; f < 4; ++f)
		{
			TautRope::RecordedFrame Frame;
			Frame.StartLocation = Vec3(NastyAt(f), NastyAt(f + 1), NastyAt(f + 2));
			Frame.EndLocation = Vec3(NastyAt(f + 3), NastyAt(f + 4), NastyAt(f + 5));
			Frame.MaxLength = static_cast<float>(1.0 / 3.0) + static_cast<float>(f);
			Frame.DeltaTime = 1.f / 60.f;
			for (int p = 0; p < 3 + f; ++p)
			{
				TautRope::RecordedPoint Rp;
				Rp.Id = 100 * f + p;
				Rp.Location = Vec3(NastyAt(p), NastyAt(p + 6), NastyAt(p + 9));
				Rp.ShapeIndex = p % 2;
				Rp.EdgeIndex = p;
				Rp.VertIndex = TautRope::IndexNone;
				Frame.Capture.AfterMovement.push_back(Rp);
				Rp.Id += 1000;
				Frame.Capture.AfterCollision.push_back(Rp);
				Rp.Id += 1000;
				Frame.Capture.AfterPruning.push_back(Rp);
			}
			R.Frames.push_back(Frame);
		}
		return R;
	}

	const char* TempPath() { return "tautrope_test_recording.tautrope"; }
}

// PROVES: a recording written and read back is bit-identical, awkward values and
// all.
// GUARDS: %.17g / %.9g. Sabotage: write doubles at %.15g and this goes red. The
// replay loop is worthless if the file cannot carry the state exactly.
TEST(Recording_RoundTripsBitIdentically)
{
	const TautRope::Recording Written = MakeAwkwardRecording();

	std::string Error;
	CHECK(TautRope::WriteRecording(Written, TempPath(), Error));

	TautRope::Recording Read;
	CHECK(TautRope::ReadRecording(Read, TempPath(), Error));

	// Compared through the same routine the replay verification uses, so a fault
	// in either is caught here.
	TautRope::RecordingDivergence Divergence;
	const bool bIdentical = TautRope::CompareRecordings(Written, Read, Divergence);
	CHECK(bIdentical);
	if (!bIdentical)
	{
		std::printf("      diverged at %s / %s\n", Divergence.Phase, Divergence.Field);
	}

	CHECK_EQ(Read.EngineBuild, Written.EngineBuild);
	CHECK_EQ(Read.NextPointId, Written.NextPointId);
	std::remove(TempPath());
}

// PROVES: nextpointid survives a round trip independently of the ids in use.
// FIXES: replayed point ids diverging from the editor's. The allocator had been
// reconstructed as max(id)+1, which is wrong whenever ids went to points pruned
// before the recording started. Sabotage: write 0 for nextpointid and this goes
// red.
TEST(Recording_PreservesNextPointIdSeparatelyFromPointIds)
{
	// The allocator cannot be inferred from the surviving points: ids belonging
	// to points pruned before recording started have already advanced it. Getting
	// this wrong was a real bug, caught by replay verification.
	TautRope::Recording R = MakeAwkwardRecording();
	R.InitialPoints.clear();
	R.NextPointId = 97;

	std::string Error;
	CHECK(TautRope::WriteRecording(R, TempPath(), Error));

	TautRope::Recording Read;
	CHECK(TautRope::ReadRecording(Read, TempPath(), Error));
	CHECK_EQ(Read.NextPointId, 97);
	std::remove(TempPath());
}

// PROVES: a recording from a different format version is refused, not guessed at.
// GUARDS: silent misreads. Sabotage: drop the version equality check and this
// goes red. A v1 file parsed as v2 replays with different ids and looks like a
// simulation bug.
TEST(Recording_RejectsUnknownFormatVersion)
{
	std::FILE* File = std::fopen(TempPath(), "wb");
	CHECK(File != nullptr);
	if (File != nullptr)
	{
		const char* Body = "tautrope-recording 99\nengine x\nshapes 0\ninitial 0\nnextpointid 0\nframes 0\n";
		std::fwrite(Body, 1, std::strlen(Body), File);
		std::fclose(File);
	}

	TautRope::Recording Read;
	std::string Error;
	// Rejected rather than misread: a silently misinterpreted recording would
	// produce a divergence that looks like a simulation bug.
	CHECK(!TautRope::ReadRecording(Read, TempPath(), Error));
	CHECK(!Error.empty());
	std::remove(TempPath());
}

// PROVES: verification catches a one-bit difference in a single coordinate, and
// reports the frame, point, phase and field.
// GUARDS: the whole loop's foundation -- a comparison that tolerates small
// differences would report PASS on a replay that had already drifted. Sabotage:
// make SameBits(double) return true and this goes red.
TEST(Compare_DetectsSingleBitDifferenceInOneCoordinate)
{
	const TautRope::Recording A = MakeAwkwardRecording();
	TautRope::Recording B = A;

	double& Target = B.Frames[2].Capture.AfterCollision[1].Location.Y;
	std::uint64_t Bits = 0;
	std::memcpy(&Bits, &Target, sizeof(Bits));
	Bits += 1;
	std::memcpy(&Target, &Bits, sizeof(Target));

	TautRope::RecordingDivergence Divergence;
	CHECK(!TautRope::CompareRecordings(A, B, Divergence));
	CHECK_EQ(Divergence.FrameIndex, 2);
	CHECK_EQ(Divergence.PointIndex, 1);
	CHECK_STR_EQ(Divergence.Phase, "collision");
	CHECK_STR_EQ(Divergence.Field, "location.y");
	// One frame, not a cascade: the count distinguishes the two.
	CHECK_EQ(Divergence.DivergentFrameCount, 1);
}

// PROVES: identity is compared, not just position. Two points in the same place
// with different ids are a divergence.
// GUARDS: id drift, which is how the nextpointid bug first showed itself.
TEST(Compare_DetectsPointIdDifference)
{
	const TautRope::Recording A = MakeAwkwardRecording();
	TautRope::Recording B = A;
	B.Frames[0].Capture.AfterPruning[0].Id += 1;

	TautRope::RecordingDivergence Divergence;
	CHECK(!TautRope::CompareRecordings(A, B, Divergence));
	CHECK_STR_EQ(Divergence.Field, "id");
}

// PROVES: verification does not cry wolf on a genuinely identical pair.
// GUARDS: the other direction of the same comparison. A checker that always
// reported divergence would be just as useless, and every PASS in this repo is
// evidence only if this holds.
TEST(Compare_AcceptsIdenticalRecordings)
{
	const TautRope::Recording A = MakeAwkwardRecording();
	TautRope::RecordingDivergence Divergence;
	CHECK(TautRope::CompareRecordings(A, A, Divergence));
	CHECK_EQ(Divergence.DivergentFrameCount, 0);
	CHECK(!Divergence.bDiverged);
}
