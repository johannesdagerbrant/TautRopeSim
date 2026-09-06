#pragma once

#include "TautRopeCore/Core.h"

#include <cmath>
#include <vector>

// These types mirror the UE originals the simulation was written against.
// Vec3 is double to match FVector in UE5, and the float/double mix at every call
// site is preserved deliberately -- widening a float there changes results.
namespace TautRope
{
	// Mirrors KINDA_SMALL_NUMBER, UE_SMALL_NUMBER and MAX_FLT.
	inline constexpr float KindaSmallNumber = 1.e-4f;
	inline constexpr float SmallNumber = 1.e-8f;
	inline constexpr float MaxFloat = 3.402823466e+38f;

	struct Vec3
	{
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;

		constexpr Vec3() = default;
		constexpr Vec3(double InX, double InY, double InZ) : X(InX), Y(InY), Z(InZ) {}

		constexpr Vec3 operator+(const Vec3& V) const { return Vec3(X + V.X, Y + V.Y, Z + V.Z); }
		constexpr Vec3 operator-(const Vec3& V) const { return Vec3(X - V.X, Y - V.Y, Z - V.Z); }
		constexpr Vec3 operator-() const { return Vec3(-X, -Y, -Z); }
		constexpr Vec3 operator*(double S) const { return Vec3(X * S, Y * S, Z * S); }
		constexpr Vec3 operator/(double S) const { return Vec3(X / S, Y / S, Z / S); }

		constexpr bool operator==(const Vec3& V) const { return X == V.X && Y == V.Y && Z == V.Z; }
		constexpr bool operator!=(const Vec3& V) const { return !(*this == V); }

		constexpr double SizeSquared() const { return X * X + Y * Y + Z * Z; }
		double Size() const { return std::sqrt(SizeSquared()); }

		// Mirrors FVector::GetSafeNormal, early-out and all.
		Vec3 GetSafeNormal(double Tolerance = SmallNumber) const
		{
			const double SquareSum = SizeSquared();
			if (SquareSum == 1.0)
			{
				return *this;
			}
			if (SquareSum < Tolerance)
			{
				return Vec3();
			}
			const double Scale = 1.0 / std::sqrt(SquareSum);
			return Vec3(X * Scale, Y * Scale, Z * Scale);
		}

		static constexpr double Dot(const Vec3& A, const Vec3& B)
		{
			return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
		}

		static constexpr Vec3 Cross(const Vec3& A, const Vec3& B)
		{
			return Vec3(
				A.Y * B.Z - A.Z * B.Y
				, A.Z * B.X - A.X * B.Z
				, A.X * B.Y - A.Y * B.X
			);
		}

		static double Dist(const Vec3& A, const Vec3& B) { return (B - A).Size(); }
	};

	constexpr Vec3 operator*(double S, const Vec3& V) { return V * S; }

	struct Quat
	{
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;
		double W = 1.0;

		constexpr Quat() = default;
		constexpr Quat(double InX, double InY, double InZ, double InW) : X(InX), Y(InY), Z(InZ), W(InW) {}

		// Mirrors FQuat::RotateVector, including operation order.
		constexpr Vec3 RotateVector(const Vec3& V) const
		{
			const Vec3 Q(X, Y, Z);
			const Vec3 T = 2.0 * Vec3::Cross(Q, V);
			return V + (W * T) + Vec3::Cross(Q, T);
		}

		constexpr Vec3 GetForwardVector() const { return RotateVector(Vec3(1.0, 0.0, 0.0)); }
		constexpr Vec3 GetUpVector() const { return RotateVector(Vec3(0.0, 0.0, 1.0)); }
	};

	struct Int2
	{
		int32 X = 0;
		int32 Y = 0;

		constexpr Int2() = default;
		constexpr Int2(int32 InX, int32 InY) : X(InX), Y(InY) {}

		constexpr bool operator==(const Int2& V) const { return X == V.X && Y == V.Y; }
		constexpr bool operator!=(const Int2& V) const { return !(*this == V); }
	};

	namespace Math
	{
		// Mirrors FMath::Clamp's exact comparison order.
		template <typename T>
		constexpr T Clamp(const T Value, const T Min, const T Max)
		{
			return (Value < Min) ? Min : (Value < Max) ? Value : Max;
		}

		template <typename T>
		constexpr T Abs(const T Value)
		{
			return (Value < T(0)) ? -Value : Value;
		}

		template <typename T>
		constexpr bool IsNearlyZero(const T Value, const T Tolerance)
		{
			return Abs(Value) <= Tolerance;
		}

		// Mirrors FMath::Lerp: A + Alpha * (B - A).
		constexpr Vec3 Lerp(const Vec3& A, const Vec3& B, const double Alpha)
		{
			return A + Alpha * (B - A);
		}
	}

	template <typename T>
	inline bool Contains(const std::vector<T>& Container, const T& Item)
	{
		for (const T& Element : Container)
		{
			if (Element == Item)
			{
				return true;
			}
		}
		return false;
	}

	template <typename T>
	inline int32 Num(const std::vector<T>& Container)
	{
		return static_cast<int32>(Container.size());
	}
}
