#include "Framework.h"

namespace TautRopeTest
{
	namespace
	{
		int GFailureCount = 0;
	}

	std::vector<TestCase>& Registry()
	{
		static std::vector<TestCase> Tests;
		return Tests;
	}

	void ReportFailure(const char* File, int Line, const std::string& Message)
	{
		++GFailureCount;
		std::printf("    %s:%d\n      %s\n", File, Line, Message.c_str());
	}

	std::string Describe(double Value)
	{
		char Buffer[64];
		std::snprintf(Buffer, sizeof(Buffer), "%.17g", Value);
		return Buffer;
	}

	std::string Describe(float Value)
	{
		char Buffer[64];
		std::snprintf(Buffer, sizeof(Buffer), "%.9g", Value);
		return Buffer;
	}

	std::string Describe(int Value)
	{
		char Buffer[32];
		std::snprintf(Buffer, sizeof(Buffer), "%d", Value);
		return Buffer;
	}

	std::string Describe(long long Value)
	{
		char Buffer[32];
		std::snprintf(Buffer, sizeof(Buffer), "%lld", Value);
		return Buffer;
	}

	std::string Describe(std::size_t Value)
	{
		char Buffer[32];
		std::snprintf(Buffer, sizeof(Buffer), "%zu", Value);
		return Buffer;
	}

	std::string Describe(const std::string& Value)
	{
		return Value;
	}

	std::string Describe(bool Value)
	{
		return Value ? "true" : "false";
	}

	void CheckTrue(const char* File, int Line, const char* Expression, bool Condition)
	{
		if (!Condition)
		{
			ReportFailure(File, Line, std::string("CHECK(") + Expression + ") failed");
		}
	}

	void CheckStringEq(const char* File, int Line, const char* ActualExpr, const char* ExpectedExpr, const char* Actual, const char* Expected)
	{
		const bool bEqual = Actual != nullptr && Expected != nullptr && std::strcmp(Actual, Expected) == 0;
		if (!bEqual)
		{
			ReportFailure(File, Line,
				std::string(ActualExpr) + " equals " + ExpectedExpr
				+ "\n      actual   " + Describe(Actual)
				+ "\n      expected " + Describe(Expected));
		}
	}

	std::string Describe(const char* Value)
	{
		return Value != nullptr ? Value : "(null)";
	}

	int RunAll(const char* Filter)
	{
		int Passed = 0;
		int Failed = 0;
		int PendingPassed = 0;
		int PendingFailed = 0;

		for (const TestCase& Test : Registry())
		{
			if (Filter != nullptr && std::strstr(Test.Name, Filter) == nullptr)
			{
				continue;
			}

			GFailureCount = 0;
			std::printf("%-12s %s\n", Test.bPending ? "[pending]" : "[run]", Test.Name);
			Test.Body();

			if (Test.bPending)
			{
				if (GFailureCount == 0) { ++PendingPassed; } else { ++PendingFailed; }
			}
			else
			{
				if (GFailureCount == 0) { ++Passed; } else { ++Failed; }
			}
		}

		std::printf("\n%d passed, %d failed", Passed, Failed);
		if (PendingPassed + PendingFailed > 0)
		{
			std::printf(", %d pending (%d of them currently reproduce the defect)",
				PendingPassed + PendingFailed, PendingFailed);
		}
		std::printf("\n");

		if (PendingPassed > 0)
		{
			std::printf(
				"\nnote: %d pending test(s) now pass. If the defect is fixed, promote\n"
				"      them from TEST_PENDING to TEST so a regression fails the suite.\n",
				PendingPassed);
		}

		return Failed == 0 ? 0 : 1;
	}
}

int main(int argc, char** argv)
{
	const char* Filter = argc > 1 ? argv[1] : nullptr;
	return TautRopeTest::RunAll(Filter);
}
