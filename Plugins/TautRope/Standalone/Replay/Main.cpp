// tautrope-replay: re-runs a recording captured by the editor through the same
// core simulation, with no engine present and no real-time pacing.
//
// The point is iteration speed: change core, rebuild core, replay, look at what
// moved. Nothing here waits for DeltaTime.
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
			"  --info      summarise the input and exit without replaying\n"
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

	// Re-runs the recorded inputs. The simulation is stateful, so the rope is
	// seeded from the recorded initial state before the first frame; starting
	// anywhere else diverges immediately.
	TautRope::Recording Replay(const TautRope::Recording& Input, double& OutSeconds)
	{
		TautRope::Rope Rope;
		Rope.AppendToNearbyShapes(Input.Shapes);
		Rope.RestoreState(Input.InitialPoints);

		TautRope::Recording Result;
		// Carried over verbatim: the header describes the engine that captured
		// the inputs, and the shapes and initial state are inputs, not outputs.
		Result.EngineBuild = Input.EngineBuild;
		Result.Shapes = Input.Shapes;
		Result.InitialPoints = Input.InitialPoints;
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

		return Result;
	}
}

int main(int argc, char** argv)
{
	const char* InputPath = nullptr;
	const char* OutputPath = nullptr;
	bool bInfoOnly = false;

	for (int Index = 1; Index < argc; ++Index)
	{
		const char* Arg = argv[Index];
		if (std::strcmp(Arg, "--info") == 0)
		{
			bInfoOnly = true;
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

	if (bInfoOnly)
	{
		return 0;
	}

	double Seconds = 0.0;
	const TautRope::Recording Result = Replay(Input, Seconds);

	std::printf("\n");
	PrintSummary(Result, "replayed");

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

	return 0;
}
