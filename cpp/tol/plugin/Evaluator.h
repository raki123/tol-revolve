#ifndef TOL_PLUGIN_EVALUATOR_H_
#define TOL_PLUGIN_EVALUATOR_H_

#include <boost/shared_ptr.hpp>

#include <brain/Evaluator.h>

namespace tol {

class Evaluator
        : public revolve::brain::Evaluator
{
public:
    virtual double
    fitness();

    virtual void
    start();
};

typedef boost::shared_ptr<tol::Evaluator> EvaluatorPtr;

}

#endif // TOL_PLUGIN_EVALUATOR_H_
