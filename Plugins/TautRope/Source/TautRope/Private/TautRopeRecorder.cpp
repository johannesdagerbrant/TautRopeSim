#include "TautRopeRecorder.h"

#include "HAL/FileManager.h"
#include "Misc/DateTime.h"
#include "Misc/EngineVersion.h"
#include "Misc/Paths.h"
#include "TautRopeCore/Rope.h"

DEFINE_LOG_CATEGORY_STATIC(LogTautRopeRecorder, Log, All);

static TAutoConsoleVariable<int32> CVarRecord(
	TEXT("tautrope.record"),
	0,
	TEXT("Record the rope simulation for headless replay.\n")
	TEXT("1: start capturing to memory\n")
	TEXT("0: stop and write the recording to Saved/TautRopeRecordings"),
	ECVF_Cheat
);

namespace
{
	// Frames are appended every tick, so the buffer is sized up front rather than
	// growing from nothing while the simulation is being measured.
	constexpr int32 InitialFrameCapacity = 8192;

	FString MakeRecordingPath(const FString& Label)
	{
		const FString Directory = FPaths::ProjectSavedDir() / TEXT("TautRopeRecordings");
		IFileManager::Get().MakeDirectory(*Directory, true);

		const FString Stamp = FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S"));
		const FString Name = Label.IsEmpty() ? TEXT("TautRope") : Label;
		return Directory / FString::Printf(TEXT("%s_%s.tautrope"), *Name, *Stamp);
	}

	TautRope::Vec3 ToCore(const FVector& V)
	{
		return TautRope::Vec3(V.X, V.Y, V.Z);
	}
}

void FTautRopeRecorder::Start(const TautRope::Rope& Rope)
{
	Recording = TautRope::Recording();
	Recording.EngineBuild = TCHAR_TO_UTF8(*FEngineVersion::Current().ToString());

	// Shapes are static, so they are captured once rather than every frame.
	Recording.Shapes = Rope.GetNearbyShapes();

	// The simulation is stateful: a replay starting from anything but the state
	// entering this frame diverges immediately.
	TautRope::CapturePoints(Rope.GetPoints(), Recording.InitialPoints);

	Recording.Frames.reserve(InitialFrameCapacity);

	bIsRecording = true;
	UE_LOG(LogTautRopeRecorder, Display,
		TEXT("Recording started: %d shapes, %d initial rope points"),
		static_cast<int32>(Recording.Shapes.size()),
		static_cast<int32>(Recording.InitialPoints.size()));
}

TautRope::FrameCapture* FTautRopeRecorder::BeginFrame(
	const TautRope::Rope& Rope
	, const FVector& StartLocation
	, const FVector& EndLocation
	, float MaxLength
	, float DeltaTime
)
{
	const bool bWantsRecording = CVarRecord.GetValueOnGameThread() != 0;

	if (bWantsRecording && !bIsRecording)
	{
		Start(Rope);
	}
	else if (!bWantsRecording && bIsRecording)
	{
		Flush();
		return nullptr;
	}

	if (!bIsRecording)
	{
		return nullptr;
	}

	// Appended before the simulation runs so UpdateRope can fill the capture in
	// place; the frame is complete either way once the tick returns.
	Recording.Frames.emplace_back();
	TautRope::RecordedFrame& Frame = Recording.Frames.back();
	Frame.StartLocation = ToCore(StartLocation);
	Frame.EndLocation = ToCore(EndLocation);
	Frame.MaxLength = MaxLength;
	Frame.DeltaTime = DeltaTime;
	return &Frame.Capture;
}

void FTautRopeRecorder::Flush()
{
	if (!bIsRecording)
	{
		return;
	}
	bIsRecording = false;

	const FString Path = MakeRecordingPath(Label);
	std::string Error;
	if (TautRope::WriteRecording(Recording, TCHAR_TO_UTF8(*Path), Error))
	{
		UE_LOG(LogTautRopeRecorder, Display,
			TEXT("Recording written: %s (%d frames)"),
			*Path, static_cast<int32>(Recording.Frames.size()));
	}
	else
	{
		UE_LOG(LogTautRopeRecorder, Error,
			TEXT("Failed to write recording: %s"), UTF8_TO_TCHAR(Error.c_str()));
	}

	Recording = TautRope::Recording();
}
