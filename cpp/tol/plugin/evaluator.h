#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <brain/evaluator.h>
#include <boost/shared_ptr.hpp>

namespace tol {

class Evaluator : public revolve::brain::Evaluator
{
public:
virtual double fitness();
virtual void start();
};

typedef boost::shared_ptr< tol::Evaluator > EvaluatorPtr;

}

#endif // EVALUATOR_H
