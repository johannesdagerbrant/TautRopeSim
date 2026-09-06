#include "TautRopeShapeSerialization.h"

#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"

namespace
{
	// Bumped only on a format change; a blob written by an older version is
	// discarded rather than misread.
	constexpr int32 ShapeBlobVersion = 1;

	template <typename ArchiveType>
	void SerializeShapes(ArchiveType& Ar, std::vector<TautRope::CollisionShape>& Shapes)
	{
		int32 ShapeCount = static_cast<int32>(Shapes.size());
		Ar << ShapeCount;
		if (Ar.IsLoading())
		{
			Shapes.clear();
			Shapes.resize(static_cast<std::size_t>(FMath::Max(ShapeCount, 0)));
		}

		for (TautRope::CollisionShape& Shape : Shapes)
		{
			int32 VertexCount = static_cast<int32>(Shape.Vertices.size());
			Ar << VertexCount;
			if (Ar.IsLoading())
			{
				Shape.Vertices.resize(static_cast<std::size_t>(FMath::Max(VertexCount, 0)));
			}
			for (TautRope::Vec3& Vertex : Shape.Vertices)
			{
				Ar << Vertex.X << Vertex.Y << Vertex.Z;
			}

			int32 EdgeCount = static_cast<int32>(Shape.Edges.size());
			Ar << EdgeCount;
			if (Ar.IsLoading())
			{
				Shape.Edges.resize(static_cast<std::size_t>(FMath::Max(EdgeCount, 0)));
			}
			for (TautRope::Int2& Edge : Shape.Edges)
			{
				Ar << Edge.X << Edge.Y;
			}

			int32 VertToEdgeCount = static_cast<int32>(Shape.VertToEdges.size());
			Ar << VertToEdgeCount;
			if (Ar.IsLoading())
			{
				Shape.VertToEdges.resize(static_cast<std::size_t>(FMath::Max(VertToEdgeCount, 0)));
			}
			for (std::vector<TautRope::int32>& Adjacent : Shape.VertToEdges)
			{
				int32 AdjacentCount = static_cast<int32>(Adjacent.size());
				Ar << AdjacentCount;
				if (Ar.IsLoading())
				{
					Adjacent.resize(static_cast<std::size_t>(FMath::Max(AdjacentCount, 0)));
				}
				for (TautRope::int32& EdgeIndex : Adjacent)
				{
					Ar << EdgeIndex;
				}
			}

			int32 RotationCount = static_cast<int32>(Shape.EdgeRotations.size());
			Ar << RotationCount;
			if (Ar.IsLoading())
			{
				Shape.EdgeRotations.resize(static_cast<std::size_t>(FMath::Max(RotationCount, 0)));
			}
			for (TautRope::Quat& Rotation : Shape.EdgeRotations)
			{
				Ar << Rotation.X << Rotation.Y << Rotation.Z << Rotation.W;
			}

			int32 CornerCount = static_cast<int32>(Shape.IsCornerVertexList.size());
			Ar << CornerCount;
			if (Ar.IsLoading())
			{
				Shape.IsCornerVertexList.resize(static_cast<std::size_t>(FMath::Max(CornerCount, 0)));
			}
			for (std::size_t Index = 0; Index < Shape.IsCornerVertexList.size(); ++Index)
			{
				// std::vector<bool> is a bit proxy, so it cannot be serialized by
				// reference like the others.
				uint8 Value = Shape.IsCornerVertexList[Index] ? 1 : 0;
				Ar << Value;
				if (Ar.IsLoading())
				{
					Shape.IsCornerVertexList[Index] = Value != 0;
				}
			}
		}
	}
}

namespace TautRopeShapeSerialization
{
	void Save(const std::vector<TautRope::CollisionShape>& Shapes, TArray<uint8>& OutBytes)
	{
		OutBytes.Reset();
		FMemoryWriter Writer(OutBytes);

		int32 Version = ShapeBlobVersion;
		Writer << Version;

		SerializeShapes(Writer, const_cast<std::vector<TautRope::CollisionShape>&>(Shapes));
	}

	void Load(std::vector<TautRope::CollisionShape>& OutShapes, const TArray<uint8>& Bytes)
	{
		OutShapes.clear();
		if (Bytes.Num() < static_cast<int32>(sizeof(int32)))
		{
			return;
		}

		FMemoryReader Reader(Bytes);

		int32 Version = 0;
		Reader << Version;
		if (Version != ShapeBlobVersion)
		{
			return;
		}

		SerializeShapes(Reader, OutShapes);
	}
}
