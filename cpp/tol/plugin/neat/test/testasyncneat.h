#ifndef TESTASYNCNEAT_H
#define TESTASYNCNEAT_H

/**
 * Test class for AsyncNeat class
 */
class TestAsyncNeat
{
public:
    TestAsyncNeat();
    ~TestAsyncNeat();
    /**
     * Runs all tests. Returns false if one of the tests fails.
     */
    bool test();

private:
    /**
     * test if the algorithm is able to resolve the XOR problem
     */
    bool testXOR();
    const int MAX_EVALUATIONS = 9999;
};

#endif // TESTASYNCNEAT_H
