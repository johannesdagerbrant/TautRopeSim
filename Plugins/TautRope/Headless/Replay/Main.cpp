// tautrope-replay: re-runs a recording captured by the editor through the same
// core simulation, with no engine present and no real-time pacing.
//
// The point is iteration speed: change core, rebuild core, replay, look at what
// moved. Nothing here waits for DeltaTime.
#include "TautRopeCore/Config.h"
#include "TautRopeCore/Analysis.h"
#include "TautRopeCore/Compare.h"
#include "TautRopeCore/Recording.h"
#include "TautRopeCore/Rope.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

namespace
{
	void PrintUsage()
	{
		std::printf(
			"usage: tautrope-replay [options] <recording.tautrope>\n"
			"\n"
			"  Replays the recording through the core simulation and reports what\n"
			"  it produced. Runs as fast as the CPU allows.\n"
			"\n"
			"options:\n"
			"  -o <path>   write the replayed result as a recording\n"
			"  --verify    compare the replay against the captured output\n"
			"  --info      summarise the input and exit without replaying\n"
			"  --analyse <what>   measure the recording without replaying;\n"
			"              what = penetration | edges | vertex | slides | onsets | ties | conditioning | all\n"
			"\n"
			"exit codes:\n"
			"  0 ok   1 error   2 usage   3 verify failed\n"
		);
	}

	struct CountRange
	{
		int Min = 0;
		int Max = 0;
		bool bHasAny = false;

		void Add(const std::size_t Count)
		{
			const int Value = static_cast<int>(Count);
			if (!bHasAny)
			{
				Min = Value;
				Max = Value;
				bHasAny = true;
				return;
			}
			if (Value < Min) { Min = Value; }
			if (Value > Max) { Max = Value; }
		}
	};

	void PrintSummary(const TautRope::Recording& Recording, const char* Label)
	{
		std::size_t TotalVertices = 0;
		std::size_t TotalEdges = 0;
		for (const TautRope::CollisionShape& Shape : Recording.Shapes)
		{
			TotalVertices += Shape.Vertices.size();
			TotalEdges += Shape.Edges.size();
		}

		std::printf("%s\n", Label);
		std::printf("  engine      %s\n", Recording.EngineBuild.c_str());
		std::printf("  shapes      %zu (%zu vertices, %zu edges)\n",
			Recording.Shapes.size(), TotalVertices, TotalEdges);
		std::printf("  initial     %zu rope points\n", Recording.InitialPoints.size());
		std::printf("  frames      %zu\n", Recording.Frames.size());

		if (Recording.Frames.empty())
		{
			return;
		}

		CountRange Movement;
		CountRange Collision;
		CountRange Pruning;
		double TotalDeltaTime = 0.0;
		for (const TautRope::RecordedFrame& Frame : Recording.Frames)
		{
			Movement.Add(Frame.Capture.AfterMovement.size());
			Collision.Add(Frame.Capture.AfterCollision.size());
			Pruning.Add(Frame.Capture.AfterPruning.size());
			TotalDeltaTime += Frame.DeltaTime;
		}

		std::printf("  captured    %.3f s of DeltaTime\n", TotalDeltaTime);
		std::printf("  rope points per frame, min..max\n");
		std::printf("    movement  %d..%d\n", Movement.Min, Movement.Max);
		std::printf("    collision %d..%d\n", Collision.Min, Collision.Max);
		std::printf("    pruning   %d..%d\n", Pruning.Min, Pruning.Max);

		if (Collision.Max <= 2 && Pruning.Max <= 2)
		{
			std::printf(
				"  note: the rope never gained an intermediate point, so this\n"
				"        recording exercises only the straight-line case.\n"
			);
		}
	}

	void PrintPenetration(const TautRope::Recording& R)
	{
		const TautRope::PenetrationReport P = TautRope::AnalysePenetration(R);
		std::printf("penetration\n");
		if (P.bAnyShapeUnusable)
		{
			std::printf("  warning: at least one shape yielded fewer than 4 face planes,\n"
				"           so its numbers below are not trustworthy\n");
		}
		for (std::size_t i = 0; i < R.Shapes.size(); ++i)
		{
			const TautRope::ShapePlanes Pl = TautRope::FindShapePlanes(R.Shapes[i]);
			std::printf("  shape %zu: %zu face planes from %zu in-face edges\n",
				i, Pl.Points.size(), Pl.InFaceEdges.size());
		}
		std::printf("  frames affected   %d of %d\n", P.FramesAffected, P.FramesCompared);
		if (P.FirstFrame == TautRope::IndexNone)
		{
			std::printf("  no rope segment passes through a shape\n");
			return;
		}
		std::printf("  first at frame    %d\n", P.FirstFrame);
		std::printf("  worst rope line   %.3f units inside shape %d at frame %d (point ids %d and %d)\n",
			P.PeakSegmentLength, P.PeakSegmentShape, P.PeakSegmentFrame, P.PeakSegmentPointA, P.PeakSegmentPointB);
		std::printf("  deepest point     %.3f units inside, point id %d at frame %d\n",
			P.PeakPointDepth, P.PeakPointId, P.PeakPointFrame);
		std::printf("  at the last frame %.3f units of rope still inside\n", P.FinalSegmentLength);
	}

	void PrintEdges(const TautRope::Recording& R)
	{
		const TautRope::EdgeUsageReport E = TautRope::AnalyseEdgeUsage(R);
		std::printf("edges\n");
		std::printf("  %d edges across %zu shapes, of which %d lie flat across a face\n",
			E.TotalEdges, R.Shapes.size(), E.InFaceEdges);
		for (std::size_t i = 0; i < R.Shapes.size(); ++i)
		{
			const TautRope::ShapePlanes Pl = TautRope::FindShapePlanes(R.Shapes[i]);
			std::printf("  shape %zu in-face edges:", i);
			for (const TautRope::int32 EdgeIndex : Pl.InFaceEdges)
			{
				std::printf(" %d", EdgeIndex);
			}
			std::printf("\n");
		}
		std::printf("  points created during the recording: %d\n", E.PointsBorn);
		std::printf("  born already on an in-face edge:     %d (%.1f%%)\n",
			E.PointsBornOnInFaceEdge,
			100.0 * E.PointsBornOnInFaceEdge / (E.PointsBorn > 0 ? E.PointsBorn : 1));
		std::printf("  born on the same frame as another:   %d\n", E.PointsBornOnSameFrameAsAnother);
		if (E.FirstBornOnInFaceFrame != TautRope::IndexNone)
		{
			std::printf("  first such birth at frame %d: point id %d\n",
				E.FirstBornOnInFaceFrame, E.FirstBornOnInFacePointId);
		}
		std::printf("  rope points attached to an edge: %d point-frames\n", E.AttachedPointFrames);
		std::printf("  of those, on an in-face edge:    %d (%.1f%%)\n",
			E.PointFramesOnInFaceEdges,
			100.0 * E.PointFramesOnInFaceEdges / (E.AttachedPointFrames > 0 ? E.AttachedPointFrames : 1));
		if (E.FirstFrameOnInFaceEdge != TautRope::IndexNone)
		{
			std::printf("  first at frame %d: point id %d on shape %d edge %d\n",
				E.FirstFrameOnInFaceEdge, E.FirstPointIdOnInFaceEdge,
				E.FirstInFaceShapeIndex, E.FirstInFaceEdgeIndex);
			std::printf("  a point resting on an in-face edge sits in the middle of a flat\n"
				"  face, so the rope line to its neighbour can pass through the solid\n");
		}
	}



	void PrintOnsets(const TautRope::Recording& R)
	{
		const TautRope::OnsetReport O = TautRope::AnalyseOnsets(R);
		std::printf("penetration onsets\n");
		std::printf("  already inside after movement  %d\n", O.BlamedOnMovement);
		std::printf("  first inside after collision   %d\n", O.BlamedOnCollision);
		std::printf("  only inside after pruning      %d\n", O.BlamedOnPruning);
		if (O.Onsets.empty())
		{
			std::printf("  the rope never goes from clean to inside a shape\n");
			return;
		}
		std::printf("\n  frame   after move  after coll  after prune  +ins  -rem  blame\n");
		for (const TautRope::PenetrationOnset& E : O.Onsets)
		{
			const char* Blame = E.AfterMovement > 1.0 ? "movement"
				: (E.AfterCollision > 1.0 ? "collision" : "pruning");
			std::printf("  %-7d %10.3f  %10.3f  %11.3f  %4d  %4d  %s\n",
				E.Frame, E.AfterMovement, E.AfterCollision, E.AfterPruning,
				E.Inserted, E.Removed, Blame);
		}
	}
	void PrintSlides(const TautRope::Recording& R)
	{
		const TautRope::SlideReport S = TautRope::AnalyseSlides(R);
		std::printf("slide-off events\n");
		std::printf("  frames that removed points   %d\n", S.FramesWithRemoval);
		std::printf("  points removed in total      %d\n", S.TotalPointsRemoved);
		std::printf("  removed group shared a vertex %d\n", S.EventsAtSharedVertex);
		std::printf("  events that began penetration %d\n", S.EventsThatBeganPenetration);
		if (S.Events.empty())
		{
			std::printf("  the rope never lost a point, so it never slid over anything\n");
			return;
		}
		std::printf("\n  frame  pts  -n  shape/vert  spread   inside b/a        worst inside within 30f\n");
		for (const TautRope::SlideEvent& E : S.Events)
		{
			std::printf("  %-6d %-4d %-3d %-5d ", E.Frame, E.PointsBefore, E.PointsRemoved, E.PointsAdded);
			if (E.SharedVertIndex != TautRope::IndexNone)
			{
				std::printf("%d/v%-8d ", E.SharedShapeIndex, E.SharedVertIndex);
			}
			else
			{
				std::printf("%-11s ", "-");
			}
			std::printf("%7.3f  %7.3f / %-7.3f  %8.3f @ frame %-6d", E.SpreadBefore, E.InsideBefore, E.InsideAfter, E.InsideWithinWindow, E.InsideWindowFrame);
			std::printf("  removed from shape");
			for (const TautRope::int32 Sh : E.RemovedFromShapes) { std::printf(" %d", Sh); }
			if (E.PenetratedShape != TautRope::IndexNone && E.InsideAfter > 1.0)
			{
				std::printf(", inside shape %d", E.PenetratedShape);
			}
			if (E.bAllRemovedOnInFaceEdge)      { std::printf("  all in-face"); }
			else if (E.bAnyRemovedOnInFaceEdge) { std::printf("  some in-face"); }
			std::printf("\n");
		}
	}

	void PrintVertexApproaches(const TautRope::Recording& R)
	{
		const std::vector<TautRope::VertexApproach> V = TautRope::AnalyseVertexApproaches(R);
		std::printf("vertex approaches\n");
		if (V.empty())
		{
			std::printf("  no adjacent points sharing a vertex for long enough to measure\n");
			return;
		}
		for (const TautRope::VertexApproach& A : V)
		{
			std::printf("  ids %d/%d on shape %d vertex %d, frames %d..%d\n",
				A.PointIdA, A.PointIdB, A.ShapeIndex, A.VertIndex, A.FirstFrame, A.LastFrame);
			std::printf("    distance %.3f -> %.3f, closing at %.8f units/frame\n",
				A.StartDistance, A.EndDistance, A.FinalSpeed);
			if (A.bEndedByRemoval)
			{
				std::printf("    ARRIVED: pruned at frame %d after closing to %.3f\n",
					A.LastFrame + 1, A.EndDistance);
			}
			else if (A.FinalSpeed > 0.0)
			{
				std::printf("    still closing at the end of the recording; do not read the rate\n"
					"    as a countdown, arrival is a prune, see --analyse slides\n");
			}
			else
			{
				std::printf("    not closing at all over the measured tail\n");
			}
		}
	}


	void PrintTies(const TautRope::Recording& R)
	{
		const TautRope::TiedSweepReport T = TautRope::AnalyseTiedSweeps(R);
		std::printf("tied sweeps\n");
		std::printf("  sweep triangles examined    %lld\n", T.Sweeps);
		std::printf("  produced at least one hit   %lld\n", T.SweepsWithHit);
		std::printf("  sweeps hitting 2+ edges     %lld\n", T.SweepsWithMultipleHits);
		std::printf("    in-face edge won the sweep  %lld\n", T.InFaceEdgeWon);
		std::printf("    in-face edge also reported  %lld\n", T.InFaceEdgeAlsoReported);
		std::printf("    ratio gap to runner-up: zero %lld, <0.001 %lld, larger %lld\n",
			T.GapExactlyZero, T.GapUnderMilli, T.GapOverMilli);
		std::printf("  two edges, same sweep ratio %lld\n", T.SweepsWithTie);
		std::printf("    involving an in-face edge %lld\n", T.TiesInvolvingInFaceEdge);
		std::printf("    the two edges share a vertex %lld\n", T.TiesAtSharedVertex);
		std::printf("    and meet at the same point   %lld\n", T.TiesAtSameLocation);
		std::printf("    across two different shapes  %lld\n", T.TiesAcrossShapes);
		if (T.FirstTieFrame != TautRope::IndexNone)
		{
			std::printf("  first at frame %d: shape %d edges %d and %d, shared vertex %d\n",
				T.FirstTieFrame, T.FirstTieShape, T.FirstTieEdgeA, T.FirstTieEdgeB, T.FirstTieSharedVert);
			std::printf("  SweepSegmentTriangleAgainstShape keeps one hit and compares with a\n"
				"  strict <, so on a tie the lower edge index wins and the other edge is\n"
				"  never reported to the collision phase at all\n");
		}

	}
	void PrintConditioning(const TautRope::Recording& R)
	{
		const TautRope::ConditioningReport C = TautRope::AnalyseConditioning(R);
		std::printf("sweep conditioning\n");
		std::printf("  sweep/edge tests            %lld\n", C.Tests);
		std::printf("  well conditioned, accepted  %lld\n", C.WellConditionedAccepted);
		std::printf("  near coplanar               %lld\n", C.NearCoplanar);
		std::printf("  near coplanar, accepted     %lld\n", C.NearCoplanarAccepted);
		std::printf("  near coplanar, in-face edge %lld\n", C.NearCoplanarOnInFaceEdge);
		std::printf("  Det bitwise zero            %lld\n", C.ExactlyZero);
		std::printf("  conditioning by decade\n");
		for (int i = 0; i < TautRope::ConditioningDecades; ++i)
		{
			if (C.Decade[i] == 0)
			{
				continue;
			}
			if (i == TautRope::ConditioningDecades - 1)
			{
				std::printf("    <1e-%-2d                   %lld\n", i, C.Decade[i]);
			}
			else
			{
				std::printf("    1e-%-2d .. 1e-%-2d            %lld\n", i + 1, i, C.Decade[i]);
			}
		}
		if (C.NearCoplanar > 0 && C.NearCoplanarAccepted == 0)
		{
			std::printf("  every coplanar sweep was rejected: the rope slides flat across\n"
				"  those edges without registering them\n");
		}
	}

	// Re-runs the recorded inputs. The simulation is stateful, so the rope is
	// seeded from the recorded initial state before the first frame; starting
	// anywhere else diverges immediately.
	TautRope::Recording Replay(const TautRope::Recording& Input, double& OutSeconds, int& OutCapHits, int& OutMostIterations, int& OutRemoveCapHits, int& OutMostRemoveSweep)
	{
		TautRope::Rope Rope;
		Rope.AppendToNearbyShapes(Input.Shapes);
		Rope.RestoreState(Input.InitialPoints, Input.NextPointId);

		TautRope::Recording Result;
		// Carried over verbatim: the header describes the engine that captured
		// the inputs, and the shapes and initial state are inputs, not outputs.
		Result.EngineBuild = Input.EngineBuild;
		Result.Shapes = Input.Shapes;
		Result.InitialPoints = Input.InitialPoints;
		Result.NextPointId = Input.NextPointId;
		Result.Frames.resize(Input.Frames.size());

		const auto Start = std::chrono::steady_clock::now();
		for (std::size_t Index = 0; Index < Input.Frames.size(); ++Index)
		{
			const TautRope::RecordedFrame& In = Input.Frames[Index];
			TautRope::RecordedFrame& Out = Result.Frames[Index];

			Out.StartLocation = In.StartLocation;
			Out.EndLocation = In.EndLocation;
			Out.MaxLength = In.MaxLength;
			Out.DeltaTime = In.DeltaTime;

			Rope.UpdateRope(In.StartLocation, In.EndLocation, In.MaxLength, nullptr, &Out.Capture);
		}
		const auto End = std::chrono::steady_clock::now();
		OutSeconds = std::chrono::duration<double>(End - Start).count();
		OutCapHits = Rope.CollisionIterationCapHits;
		OutMostIterations = Rope.MostCollisionIterations;
		OutRemoveCapHits = Rope.RemoveSweepIterationCapHits;
		OutMostRemoveSweep = Rope.MostRemoveSweepIterations;

		return Result;
	}
}

int main(int argc, char** argv)
{
	const char* InputPath = nullptr;
	const char* OutputPath = nullptr;
	bool bInfoOnly = false;
	bool bVerify = false;
	const char* Analyse = nullptr;

	for (int Index = 1; Index < argc; ++Index)
	{
		const char* Arg = argv[Index];
		if (std::strcmp(Arg, "--info") == 0)
		{
			bInfoOnly = true;
		}
		else if (std::strcmp(Arg, "--verify") == 0)
		{
			bVerify = true;
		}
		else if (std::strcmp(Arg, "--analyse") == 0 || std::strcmp(Arg, "--analyze") == 0)
		{
			if (Index + 1 >= argc)
			{
				std::fprintf(stderr, "tautrope-replay: --analyse needs one of "
					"penetration, edges, vertex, conditioning, all\n");
				return 2;
			}
			Analyse = argv[++Index];
		}
		else if (std::strcmp(Arg, "-o") == 0)
		{
			if (Index + 1 >= argc)
			{
				std::fprintf(stderr, "tautrope-replay: -o needs a path\n");
				return 2;
			}
			OutputPath = argv[++Index];
		}
		else if (Arg[0] == '-')
		{
			std::fprintf(stderr, "tautrope-replay: unknown option %s\n", Arg);
			return 2;
		}
		else if (InputPath == nullptr)
		{
			InputPath = Arg;
		}
		else
		{
			std::fprintf(stderr, "tautrope-replay: unexpected argument %s\n", Arg);
			return 2;
		}
	}

	if (InputPath == nullptr)
	{
		PrintUsage();
		return 2;
	}

	TautRope::Recording Input;
	std::string Error;
	if (!TautRope::ReadRecording(Input, InputPath, Error))
	{
		std::fprintf(stderr, "tautrope-replay: %s\n", Error.c_str());
		return 1;
	}

	PrintSummary(Input, "recorded");

	if (Analyse != nullptr)
	{
		const bool bAll = std::strcmp(Analyse, "all") == 0;
		bool bKnown = bAll;
		if (bAll || std::strcmp(Analyse, "penetration") == 0)  { std::printf("\n"); PrintPenetration(Input); bKnown = true; }
		if (bAll || std::strcmp(Analyse, "edges") == 0)        { std::printf("\n"); PrintEdges(Input); bKnown = true; }
		if (bAll || std::strcmp(Analyse, "vertex") == 0)       { std::printf("\n"); PrintVertexApproaches(Input); bKnown = true; }
		if (bAll || std::strcmp(Analyse, "slides") == 0)       { std::printf("\n"); PrintSlides(Input); bKnown = true; }
		if (bAll || std::strcmp(Analyse, "onsets") == 0)       { std::printf("\n"); PrintOnsets(Input); bKnown = true; }
		if (bAll || std::strcmp(Analyse, "ties") == 0)         { std::printf("\n"); PrintTies(Input); bKnown = true; }
		if (bAll || std::strcmp(Analyse, "conditioning") == 0) { std::printf("\n"); PrintConditioning(Input); bKnown = true; }
		if (!bKnown)
		{
			std::fprintf(stderr, "tautrope-replay: unknown analysis %s\n", Analyse);
			return 2;
		}
		return 0;
	}

	if (bInfoOnly)
	{
		return 0;
	}

	double Seconds = 0.0;
	int CapHits = 0;
	int MostIterations = 0;
	int RemoveCapHits = 0;
	int MostRemoveSweep = 0;
	const TautRope::Recording Result = Replay(Input, Seconds, CapHits, MostIterations, RemoveCapHits, MostRemoveSweep);

	std::printf("\n");
	PrintSummary(Result, "replayed");

	std::printf("\n  collision iterations, worst frame %d of %d\n",
		MostIterations, TautRope::MaxCollisionIterations);
	std::printf("  remove sweep rounds, worst frame %d of %d\n",
		MostRemoveSweep, TautRope::MaxRemoveSweepIterations);
	if (CapHits > 0 || RemoveCapHits > 0)
	{
		std::printf("  WARNING: %d frame(s) ran out of collision iterations and %d ran\n"
			"  out of remove sweep rounds. Points were still being inserted when the\n"
			"  loop gave up, which is how runaway insertion presents: not a crash,\n"
			"  just slower until it looks hung.\n", CapHits, RemoveCapHits);
	}
	std::printf("\n  replayed in %.3f s", Seconds);
	if (Seconds > 0.0)
	{
		std::printf(" (%.0f frames/s)", static_cast<double>(Result.Frames.size()) / Seconds);
	}
	std::printf("\n");

	if (OutputPath != nullptr)
	{
		if (!TautRope::WriteRecording(Result, OutputPath, Error))
		{
			std::fprintf(stderr, "tautrope-replay: %s\n", Error.c_str());
			return 1;
		}
		std::printf("  written to %s\n", OutputPath);
	}

	if (bVerify)
	{
		TautRope::RecordingDivergence Divergence;
		const bool bIdentical = TautRope::CompareRecordings(Input, Result, Divergence);

		std::printf("\nverify: %s\n", bIdentical ? "PASS" : "FAIL");
		std::printf("  frames diverged   %d of %d\n",
			Divergence.DivergentFrameCount, Divergence.ComparedFrameCount);

		if (!bIdentical)
		{
			std::printf("  first divergence\n");
			std::printf("    phase           %s\n", Divergence.Phase);
			std::printf("    field           %s\n", Divergence.Field);
			if (Divergence.FrameIndex != TautRope::IndexNone)
			{
				std::printf("    frame           %d\n", Divergence.FrameIndex);
			}
			if (Divergence.PointIndex != TautRope::IndexNone)
			{
				std::printf("    point           %d\n", Divergence.PointIndex);
			}
			std::printf("    recorded        %s\n", Divergence.RecordedValue.c_str());
			std::printf("    replayed        %s\n", Divergence.ReplayedValue.c_str());
			return 3;
		}
	}

	return 0;
}
