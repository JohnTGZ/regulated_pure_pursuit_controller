// This is to perform some function
#include "regulated_pure_pursuit_controller/helper.h"

// Bring in gtest
#include <gtest/gtest.h>

// This is to read files
#include <fstream>

/**
 * Test suites are CamelCased, like C++ types
 * Test cases are camelCased, like C++ functions
 *
 * ASSERT_* versions generate fatal failures when they fail, and abort the current function.
 * EXPECT_* versions generate nonfatal failures, which don’t abort the current function.
 *
 * *_EQ : EQUAL
 * *_NE : NOT EQUAL
 */

class TestSuite : public ::testing::Test
{
private:
    std::string mPlate;
    bool mFailService, mFailUnique;

public:
    TestSuite() : mFailService(false), mFailUnique(false)
    {
    }
    ~TestSuite() {}

    /**
     * @brief  Code here will execute just before the test starts
     *
     */
    void SetUp() override
    {
    }

    /**
     * @brief Code here will execute after the test ends
     *
     */
    void TearDown() override
    {
    }

    void failUnique() { mFailUnique = true; }
    void failService() { mFailService = true; }
};

// Test #2: Check if robot position result is invalid
TEST(TestSuite, testCase2)
{
    RegulatedPurePursuitHelper instance_(0.035887, 0.007136, 0.182558, 0.038560, 0.329162, 0.070293);
    instance_.obtainA();
    instance_.obtainB();

    // Let's check if the vectors are correct
    ASSERT_EQ(instance_.vector_a_.first, -0.146671) << "The calculation of the x value for vector A is wrong";
    ASSERT_EQ(instance_.vector_a_.second, -0.031423999999999994) << "The calculation of the y value for vector A is wrong";
    ASSERT_EQ(instance_.vector_b_.first, 0.146604) << "The calculation of the x value for vector B is wrong";
    ASSERT_EQ(instance_.vector_b_.second, 0.031733) << "The calculation of the y value for vector B is wrong";

    // Let's check if the dot product is accurate using checking up to 4 decimal places
    ASSERT_LE(instance_.dotProduct() - (-0.022499733076000002), 0.0001) << "The dot product is wrong";

    // This value should always be positive
    ASSERT_GE(instance_.magnitudeOfVector(), 0) << "The magnitude of the vector should always be positive";

    // Let's check if the cosing value is accurate
    ASSERT_LE(instance_.cosineVector() - (-0.9999977784274525), 0.001) << "The cosine calculation is wrong";

    // This should provide the final angle of 3.14
    ASSERT_LE(instance_.inverseCosineVector() - (3.14), 0.001) << "The cosine calculation is wrong";
}

TEST(TestSuite, testCase5)
{
    // std::ifstream inFile;
    // inFile.open("./test.txt");
    // int x;
    // int sum = 0;
    // if (!inFile)
    // {
    //     std::cout << "Trying to read: test.txt" << std::endl;
    //     exit(1); // terminate with error
    // }

    // while (inFile >> x)
    // {
    //     sum = sum + x;
    // }

    // inFile.close();
    // //   EXPECT_EQ(result, "") << "Test managed to identify a point on the line, the algorithm should not work this way, but no harm no foul";
    // ASSERT_FALSE(false) << "Test failed to identify malformed robot pose";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return result;
}
