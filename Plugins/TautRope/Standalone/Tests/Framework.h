#pragma once

// A deliberately tiny harness: no external dependency, so the standalone build
// stays a single cmake invocation with nothing to fetch. The agent loop runs
// this on every hypothesis, so startup cost matters more than features.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace TautRopeTest
{
	using TestBody = void (*)();

	struct TestCase
	{
		const char* Name;
		TestBody Body;
		// Known-broken behaviour. Runs and reports, but does not fail the suite,
		// so a red run always means a regression rather than a standing defect.
		bool bPending;
	};

	std::vector<TestCase>& Registry();
	void ReportFailure(const char* File, int Line, const std::string& Message);
	int RunAll(const char* Filter);

	struct Registrar
	{
		Registrar(const char* Name, TestBody Body, bool bPending)
		{
			Registry().push_back(TestCase{ Name, Body, bPending });
		}
	};

	// Bitwise, never epsilon. The whole system exists to keep results exactly
	// reproducible, so a test that tolerates drift tests the wrong thing.
	inline bool SameBits(double A, double B)
	{
		std::uint64_t BitsA = 0;
		std::uint64_t BitsB = 0;
		std::memcpy(&BitsA, &A, sizeof(BitsA));
		std::memcpy(&BitsB, &B, sizeof(BitsB));
		return BitsA == BitsB;
	}

	inline bool SameBits(float A, float B)
	{
		std::uint32_t BitsA = 0;
		std::uint32_t BitsB = 0;
		std::memcpy(&BitsA, &A, sizeof(BitsA));
		std::memcpy(&BitsB, &B, sizeof(BitsB));
		return BitsA == BitsB;
	}

	std::string Describe(double Value);
	std::string Describe(float Value);
	std::string Describe(int Value);
	std::string Describe(long long Value);
	std::string Describe(bool Value);
	std::string Describe(const char* Value);
	std::string Describe(std::size_t Value);
	std::string Describe(const std::string& Value);

	// The comparisons live in functions rather than in multi-line macros so the
	// failure reporting stays readable.
	void CheckTrue(const char* File, int Line, const char* Expression, bool Condition);
	void CheckStringEq(const char* File, int Line, const char* ActualExpr, const char* ExpectedExpr, const char* Actual, const char* Expected);

	template <typename A, typename B>
	void CheckEq(const char* File, int Line, const char* ActualExpr, const char* ExpectedExpr, const A& Actual, const B& Expected)
	{
		if (!(Actual == Expected))
		{
			ReportFailure(File, Line,
				std::string(ActualExpr) + " == " + ExpectedExpr
				+ "\n      actual   " + Describe(Actual)
				+ "\n      expected " + Describe(Expected));
		}
	}

	template <typename T>
	void CheckBits(const char* File, int Line, const char* ActualExpr, const char* ExpectedExpr, T Actual, T Expected)
	{
		if (!SameBits(Actual, Expected))
		{
			ReportFailure(File, Line,
				std::string(ActualExpr) + " bitwise == " + ExpectedExpr
				+ "\n      actual   " + Describe(Actual)
				+ "\n      expected " + Describe(Expected));
		}
	}
}

#define TAUTROPE_TEST_IMPL(Name, Pending) \
	static void Name(); \
	static TautRopeTest::Registrar Name##_Registrar(#Name, &Name, Pending); \
	static void Name()

// A normal test. Must pass.
#define TEST(Name) TAUTROPE_TEST_IMPL(Name, false)

// Documents behaviour that is known to be wrong. Reported separately and does
// not fail the suite; promote it to TEST once the defect is fixed.
#define TEST_PENDING(Name) TAUTROPE_TEST_IMPL(Name, true)

#define CHECK(Condition) TautRopeTest::CheckTrue(__FILE__, __LINE__, #Condition, (Condition))

#define CHECK_EQ(Actual, Expected) TautRopeTest::CheckEq(__FILE__, __LINE__, #Actual, #Expected, (Actual), (Expected))

// Strings compare by content: the Phase and Field labels are literals set in
// another translation unit, so pointer equality is unreliable.
#define CHECK_STR_EQ(Actual, Expected) TautRopeTest::CheckStringEq(__FILE__, __LINE__, #Actual, #Expected, (Actual), (Expected))

#define CHECK_BITS(Actual, Expected) TautRopeTest::CheckBits(__FILE__, __LINE__, #Actual, #Expected, (Actual), (Expected))
