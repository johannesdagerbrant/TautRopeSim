// std::fopen is the portable choice, but MSVC's CRT deprecates it. UE defines
// this itself; the standalone replay build does not.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
	#define _CRT_SECURE_NO_WARNINGS
#endif

#include "TautRopeCore/Recording.h"

#include "TautRopeCore/Point.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace TautRope
{
	namespace
	{
		// %.17g round-trips every double exactly; %.9g does the same for float.
		void WriteDouble(std::string& Out, const double Value)
		{
			char Buffer[64];
			std::snprintf(Buffer, sizeof(Buffer), "%.17g", Value);
			Out += Buffer;
		}

		void WriteFloat(std::string& Out, const float Value)
		{
			char Buffer[64];
			std::snprintf(Buffer, sizeof(Buffer), "%.9g", Value);
			Out += Buffer;
		}

		void WriteInt(std::string& Out, const int32 Value)
		{
			char Buffer[32];
			std::snprintf(Buffer, sizeof(Buffer), "%d", Value);
			Out += Buffer;
		}

		void WriteVec3(std::string& Out, const Vec3& V)
		{
			WriteDouble(Out, V.X);
			Out += ' ';
			WriteDouble(Out, V.Y);
			Out += ' ';
			WriteDouble(Out, V.Z);
		}

		void WriteQuat(std::string& Out, const Quat& Q)
		{
			WriteDouble(Out, Q.X);
			Out += ' ';
			WriteDouble(Out, Q.Y);
			Out += ' ';
			WriteDouble(Out, Q.Z);
			Out += ' ';
			WriteDouble(Out, Q.W);
		}

		void WritePoints(std::string& Out, const char* Tag, const std::vector<RecordedPoint>& Points)
		{
			Out += Tag;
			Out += ' ';
			WriteInt(Out, static_cast<int32>(Points.size()));
			Out += '\n';
			for (const RecordedPoint& P : Points)
			{
				Out += "p ";
				WriteInt(Out, P.Id);
				Out += ' ';
				WriteVec3(Out, P.Location);
				Out += ' ';
				WriteInt(Out, P.ShapeIndex);
				Out += ' ';
				WriteInt(Out, P.EdgeIndex);
				Out += ' ';
				WriteInt(Out, P.VertIndex);
				Out += '\n';
			}
		}

		// Minimal whitespace-delimited token reader over the whole file.
		class TokenReader
		{
		public:
			explicit TokenReader(std::string&& InText) : Text(std::move(InText)) {}

			bool Next(const char*& OutBegin, std::size_t& OutLength)
			{
				while (Pos < Text.size() && IsSpace(Text[Pos]))
				{
					++Pos;
				}
				if (Pos >= Text.size())
				{
					return false;
				}
				const std::size_t Begin = Pos;
				while (Pos < Text.size() && !IsSpace(Text[Pos]))
				{
					++Pos;
				}
				OutBegin = Text.data() + Begin;
				OutLength = Pos - Begin;
				return true;
			}

			bool Keyword(const char* Expected)
			{
				const char* Begin = nullptr;
				std::size_t Length = 0;
				if (!Next(Begin, Length))
				{
					return false;
				}
				return Length == std::strlen(Expected) && std::strncmp(Begin, Expected, Length) == 0;
			}

			bool ReadInt(int32& Out)
			{
				const char* Begin = nullptr;
				std::size_t Length = 0;
				if (!Next(Begin, Length))
				{
					return false;
				}
				Scratch.assign(Begin, Length);
				Out = static_cast<int32>(std::strtol(Scratch.c_str(), nullptr, 10));
				return true;
			}

			bool ReadDouble(double& Out)
			{
				const char* Begin = nullptr;
				std::size_t Length = 0;
				if (!Next(Begin, Length))
				{
					return false;
				}
				Scratch.assign(Begin, Length);
				Out = std::strtod(Scratch.c_str(), nullptr);
				return true;
			}

			bool ReadFloat(float& Out)
			{
				double AsDouble = 0.0;
				if (!ReadDouble(AsDouble))
				{
					return false;
				}
				Out = static_cast<float>(AsDouble);
				return true;
			}

			bool ReadVec3(Vec3& Out)
			{
				return ReadDouble(Out.X) && ReadDouble(Out.Y) && ReadDouble(Out.Z);
			}

			bool ReadQuat(Quat& Out)
			{
				return ReadDouble(Out.X) && ReadDouble(Out.Y) && ReadDouble(Out.Z) && ReadDouble(Out.W);
			}

			bool ReadWord(std::string& Out)
			{
				const char* Begin = nullptr;
				std::size_t Length = 0;
				if (!Next(Begin, Length))
				{
					return false;
				}
				Out.assign(Begin, Length);
				return true;
			}

		private:
			static bool IsSpace(const char C)
			{
				return C == ' ' || C == '\t' || C == '\r' || C == '\n';
			}

			std::string Text;
			std::string Scratch;
			std::size_t Pos = 0;
		};

		bool ReadPoints(TokenReader& Reader, const char* Tag, std::vector<RecordedPoint>& OutPoints)
		{
			if (!Reader.Keyword(Tag))
			{
				return false;
			}
			int32 Count = 0;
			if (!Reader.ReadInt(Count) || Count < 0)
			{
				return false;
			}
			OutPoints.clear();
			OutPoints.reserve(static_cast<std::size_t>(Count));
			for (int32 i = 0; i < Count; ++i)
			{
				if (!Reader.Keyword("p"))
				{
					return false;
				}
				RecordedPoint P;
				if (!Reader.ReadInt(P.Id)
					|| !Reader.ReadVec3(P.Location)
					|| !Reader.ReadInt(P.ShapeIndex)
					|| !Reader.ReadInt(P.EdgeIndex)
					|| !Reader.ReadInt(P.VertIndex))
				{
					return false;
				}
				OutPoints.push_back(P);
			}
			return true;
		}
	}

	void CapturePoints(const std::vector<Point>& Points, std::vector<RecordedPoint>& OutPoints)
	{
		OutPoints.clear();
		OutPoints.reserve(Points.size());
		for (const Point& P : Points)
		{
			RecordedPoint Out;
			Out.Id = P.Id;
			Out.Location = P.Location;
			Out.ShapeIndex = P.ShapeIndex;
			Out.EdgeIndex = P.EdgeIndex;
			Out.VertIndex = P.VertIndex;
			OutPoints.push_back(Out);
		}
	}

	bool WriteRecording(const Recording& InRecording, const char* Path, std::string& OutError)
	{
		std::string Out;
		// Recordings run to tens of megabytes; growing from nothing thrashes.
		Out.reserve(1u << 20);

		Out += "tautrope-recording ";
		WriteInt(Out, RecordingFormatVersion);
		Out += '\n';
		Out += "engine ";
		Out += InRecording.EngineBuild.empty() ? "unknown" : InRecording.EngineBuild;
		Out += '\n';

		Out += "shapes ";
		WriteInt(Out, static_cast<int32>(InRecording.Shapes.size()));
		Out += '\n';
		for (const CollisionShape& Shape : InRecording.Shapes)
		{
			Out += "vertices ";
			WriteInt(Out, static_cast<int32>(Shape.Vertices.size()));
			Out += '\n';
			for (const Vec3& V : Shape.Vertices)
			{
				Out += "v ";
				WriteVec3(Out, V);
				Out += '\n';
			}

			Out += "edges ";
			WriteInt(Out, static_cast<int32>(Shape.Edges.size()));
			Out += '\n';
			for (const Int2& E : Shape.Edges)
			{
				Out += "e ";
				WriteInt(Out, E.X);
				Out += ' ';
				WriteInt(Out, E.Y);
				Out += '\n';
			}

			Out += "vertedges ";
			WriteInt(Out, static_cast<int32>(Shape.VertToEdges.size()));
			Out += '\n';
			for (const std::vector<int32>& Adjacent : Shape.VertToEdges)
			{
				Out += "ve ";
				WriteInt(Out, static_cast<int32>(Adjacent.size()));
				for (const int32 EdgeIndex : Adjacent)
				{
					Out += ' ';
					WriteInt(Out, EdgeIndex);
				}
				Out += '\n';
			}

			Out += "edgerotations ";
			WriteInt(Out, static_cast<int32>(Shape.EdgeRotations.size()));
			Out += '\n';
			for (const Quat& Q : Shape.EdgeRotations)
			{
				Out += "q ";
				WriteQuat(Out, Q);
				Out += '\n';
			}

			Out += "corners ";
			WriteInt(Out, static_cast<int32>(Shape.IsCornerVertexList.size()));
			Out += '\n';
			for (const bool bIsCorner : Shape.IsCornerVertexList)
			{
				Out += "c ";
				WriteInt(Out, bIsCorner ? 1 : 0);
				Out += '\n';
			}
		}

		WritePoints(Out, "initial", InRecording.InitialPoints);

		Out += "frames ";
		WriteInt(Out, static_cast<int32>(InRecording.Frames.size()));
		Out += '\n';
		for (const RecordedFrame& Frame : InRecording.Frames)
		{
			Out += "frame\n";
			Out += "start ";
			WriteVec3(Out, Frame.StartLocation);
			Out += '\n';
			Out += "end ";
			WriteVec3(Out, Frame.EndLocation);
			Out += '\n';
			Out += "maxlength ";
			WriteFloat(Out, Frame.MaxLength);
			Out += '\n';
			Out += "deltatime ";
			WriteFloat(Out, Frame.DeltaTime);
			Out += '\n';
			WritePoints(Out, "movement", Frame.Capture.AfterMovement);
			WritePoints(Out, "collision", Frame.Capture.AfterCollision);
			WritePoints(Out, "pruning", Frame.Capture.AfterPruning);
		}

		std::FILE* File = std::fopen(Path, "wb");
		if (File == nullptr)
		{
			OutError = "could not open for writing: ";
			OutError += Path;
			return false;
		}
		const std::size_t Written = std::fwrite(Out.data(), 1, Out.size(), File);
		std::fclose(File);
		if (Written != Out.size())
		{
			OutError = "short write to: ";
			OutError += Path;
			return false;
		}
		return true;
	}

	bool ReadRecording(Recording& OutRecording, const char* Path, std::string& OutError)
	{
		std::FILE* File = std::fopen(Path, "rb");
		if (File == nullptr)
		{
			OutError = "could not open for reading: ";
			OutError += Path;
			return false;
		}
		std::string Text;
		char Buffer[1 << 16];
		std::size_t Read = 0;
		while ((Read = std::fread(Buffer, 1, sizeof(Buffer), File)) > 0)
		{
			Text.append(Buffer, Read);
		}
		std::fclose(File);

		TokenReader Reader(std::move(Text));
		OutRecording = Recording();

		if (!Reader.Keyword("tautrope-recording"))
		{
			OutError = "not a tautrope recording";
			return false;
		}
		int32 Version = 0;
		if (!Reader.ReadInt(Version) || Version != RecordingFormatVersion)
		{
			OutError = "unsupported recording format version";
			return false;
		}
		if (!Reader.Keyword("engine") || !Reader.ReadWord(OutRecording.EngineBuild))
		{
			OutError = "malformed engine header";
			return false;
		}

		int32 ShapeCount = 0;
		if (!Reader.Keyword("shapes") || !Reader.ReadInt(ShapeCount) || ShapeCount < 0)
		{
			OutError = "malformed shape count";
			return false;
		}
		OutRecording.Shapes.resize(static_cast<std::size_t>(ShapeCount));
		for (int32 ShapeIndex = 0; ShapeIndex < ShapeCount; ++ShapeIndex)
		{
			CollisionShape& Shape = OutRecording.Shapes[static_cast<std::size_t>(ShapeIndex)];
			int32 Count = 0;

			if (!Reader.Keyword("vertices") || !Reader.ReadInt(Count) || Count < 0)
			{
				OutError = "malformed vertices";
				return false;
			}
			Shape.Vertices.resize(static_cast<std::size_t>(Count));
			for (int32 i = 0; i < Count; ++i)
			{
				if (!Reader.Keyword("v") || !Reader.ReadVec3(Shape.Vertices[static_cast<std::size_t>(i)]))
				{
					OutError = "malformed vertex";
					return false;
				}
			}

			if (!Reader.Keyword("edges") || !Reader.ReadInt(Count) || Count < 0)
			{
				OutError = "malformed edges";
				return false;
			}
			Shape.Edges.resize(static_cast<std::size_t>(Count));
			for (int32 i = 0; i < Count; ++i)
			{
				Int2& Edge = Shape.Edges[static_cast<std::size_t>(i)];
				if (!Reader.Keyword("e") || !Reader.ReadInt(Edge.X) || !Reader.ReadInt(Edge.Y))
				{
					OutError = "malformed edge";
					return false;
				}
			}

			if (!Reader.Keyword("vertedges") || !Reader.ReadInt(Count) || Count < 0)
			{
				OutError = "malformed vertedges";
				return false;
			}
			Shape.VertToEdges.resize(static_cast<std::size_t>(Count));
			for (int32 i = 0; i < Count; ++i)
			{
				int32 AdjacentCount = 0;
				if (!Reader.Keyword("ve") || !Reader.ReadInt(AdjacentCount) || AdjacentCount < 0)
				{
					OutError = "malformed vertedge";
					return false;
				}
				std::vector<int32>& Adjacent = Shape.VertToEdges[static_cast<std::size_t>(i)];
				Adjacent.resize(static_cast<std::size_t>(AdjacentCount));
				for (int32 j = 0; j < AdjacentCount; ++j)
				{
					if (!Reader.ReadInt(Adjacent[static_cast<std::size_t>(j)]))
					{
						OutError = "malformed vertedge entry";
						return false;
					}
				}
			}

			if (!Reader.Keyword("edgerotations") || !Reader.ReadInt(Count) || Count < 0)
			{
				OutError = "malformed edgerotations";
				return false;
			}
			Shape.EdgeRotations.resize(static_cast<std::size_t>(Count));
			for (int32 i = 0; i < Count; ++i)
			{
				if (!Reader.Keyword("q") || !Reader.ReadQuat(Shape.EdgeRotations[static_cast<std::size_t>(i)]))
				{
					OutError = "malformed edge rotation";
					return false;
				}
			}

			if (!Reader.Keyword("corners") || !Reader.ReadInt(Count) || Count < 0)
			{
				OutError = "malformed corners";
				return false;
			}
			Shape.IsCornerVertexList.resize(static_cast<std::size_t>(Count));
			for (int32 i = 0; i < Count; ++i)
			{
				int32 Value = 0;
				if (!Reader.Keyword("c") || !Reader.ReadInt(Value))
				{
					OutError = "malformed corner";
					return false;
				}
				Shape.IsCornerVertexList[static_cast<std::size_t>(i)] = Value != 0;
			}
		}

		if (!ReadPoints(Reader, "initial", OutRecording.InitialPoints))
		{
			OutError = "malformed initial points";
			return false;
		}

		int32 FrameCount = 0;
		if (!Reader.Keyword("frames") || !Reader.ReadInt(FrameCount) || FrameCount < 0)
		{
			OutError = "malformed frame count";
			return false;
		}
		OutRecording.Frames.resize(static_cast<std::size_t>(FrameCount));
		for (int32 FrameIndex = 0; FrameIndex < FrameCount; ++FrameIndex)
		{
			RecordedFrame& Frame = OutRecording.Frames[static_cast<std::size_t>(FrameIndex)];
			if (!Reader.Keyword("frame")
				|| !Reader.Keyword("start") || !Reader.ReadVec3(Frame.StartLocation)
				|| !Reader.Keyword("end") || !Reader.ReadVec3(Frame.EndLocation)
				|| !Reader.Keyword("maxlength") || !Reader.ReadFloat(Frame.MaxLength)
				|| !Reader.Keyword("deltatime") || !Reader.ReadFloat(Frame.DeltaTime))
			{
				OutError = "malformed frame header";
				return false;
			}
			if (!ReadPoints(Reader, "movement", Frame.Capture.AfterMovement)
				|| !ReadPoints(Reader, "collision", Frame.Capture.AfterCollision)
				|| !ReadPoints(Reader, "pruning", Frame.Capture.AfterPruning))
			{
				OutError = "malformed frame capture";
				return false;
			}
		}

		return true;
	}
}
