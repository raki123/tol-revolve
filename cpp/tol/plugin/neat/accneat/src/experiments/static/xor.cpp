#include "staticexperiment.h"

using namespace NEAT;

static struct XorInit {
    XorInit() {
        create_static_experiment("xor", [] () {
                const real_t T = 1.0;
                const real_t F = 0.0;
                const real_t weight = 1.0;

                std::vector<Test> tests = {
                    {{
                            {{F, F}, {F}, weight},
                    }},
                    {{
                            {{F, T}, {T}, weight},
                    }},
                    {{
                            {{T, F}, {T}, weight},
                    }},
                    {{
                            {{T, T}, {F}, weight}
                    }}
                };

                return tests;
            });
    }
} init;
