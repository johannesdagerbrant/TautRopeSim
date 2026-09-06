// tautrope-replay: reads recordings captured by the editor and, once issue #10
// lands, re-runs them through the same core simulation with no engine present.
//
// Today it loads a recording and summarises it, which is enough to confirm the
// format reads outside Unreal and to see what a recording actually contains.
#include "TautRopeCore/Recording.h"

#include <cstdio>
#include <string>

namespace
{
	void PrintUsage()
	{
		std::printf(
			"usage: tautrope-replay <recording.tautrope>\n"
			"\n"
			"  Loads a recording and prints a summary.\n"
		);
	}

	struct PointCountRange
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
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		PrintUsage();
		return 2;
	}

	const char* const Path = argv[1];

	TautRope::Recording Recording;
	std::string Error;
	if (!TautRope::ReadRecording(Recording, Path, Error))
	{
		std::fprintf(stderr, "tautrope-replay: %s\n", Error.c_str());
		return 1;
	}

	std::printf("recording   %s\n", Path);
	std::printf("engine      %s\n", Recording.EngineBuild.c_str());
	std::printf("shapes      %zu\n", Recording.Shapes.size());

	std::size_t TotalVertices = 0;
	std::size_t TotalEdges = 0;
	for (const TautRope::CollisionShape& Shape : Recording.Shapes)
	{
		TotalVertices += Shape.Vertices.size();
		TotalEdges += Shape.Edges.size();
	}
	std::printf("  vertices  %zu\n", TotalVertices);
	std::printf("  edges     %zu\n", TotalEdges);

	std::printf("initial     %zu rope points\n", Recording.InitialPoints.size());
	std::printf("frames      %zu\n", Recording.Frames.size());

	PointCountRange Movement;
	PointCountRange Collision;
	PointCountRange Pruning;
	double TotalDeltaTime = 0.0;
	for (const TautRope::RecordedFrame& Frame : Recording.Frames)
	{
		Movement.Add(Frame.Capture.AfterMovement.size());
		Collision.Add(Frame.Capture.AfterCollision.size());
		Pruning.Add(Frame.Capture.AfterPruning.size());
		TotalDeltaTime += Frame.DeltaTime;
	}

	if (!Recording.Frames.empty())
	{
		std::printf("duration    %.3f s of captured DeltaTime\n", TotalDeltaTime);
		std::printf("rope points per frame (min..max)\n");
		std::printf("  movement  %d..%d\n", Movement.Min, Movement.Max);
		std::printf("  collision %d..%d\n", Collision.Min, Collision.Max);
		std::printf("  pruning   %d..%d\n", Pruning.Min, Pruning.Max);

		if (Collision.Max == Pruning.Max && Collision.Min == Pruning.Min && Collision.Max <= 2)
		{
			std::printf(
				"\nnote: the rope never gained an intermediate point, so this\n"
				"      recording exercises nothing but the straight-line case.\n"
			);
		}
	}

	return 0;
}
