#ifndef TESTMULTINNSPECIESNEAT_H
#define TESTMULTINNSPECIESNEAT_H

class TestMultiNNSpeciesNeat
{
public:
    TestMultiNNSpeciesNeat();
    ~TestMultiNNSpeciesNeat();

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

#endif // TESTMULTINNSPECIESNEAT_H
