#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <brain/evaluator.h>
#include <memory>

namespace tol {

class Evaluator : public revolve::brain::Evaluator
{
public:
virtual double fitness();
virtual void start();
};

typedef std::shared_ptr< tol::Evaluator > EvaluatorPtr;

}

#endif // EVALUATOR_H
