#include "ortools/base/commandlineflags.h"
#include "ortools/base/logging.h"
#include "ortools/constraint_solver/constraint_solver.h"

#include <vector>
#include <sstream>

// DEFINE_int32(n, 0, "Number of marks. If 0 will test different values of n.");
// DEFINE_bool(print, false, "Print solution or not?");

const int kG[] = {
  0, 1, 3, 6, 11, 17, 25, 34, 44, 55, 72, 85,
  106, 127, 151, 177, 199, 216, 246
};

const int kKnownSolutions = 19;

namespace operations_research {

void Golomb(const int n)
{
    Solver s("golomb");

    const int64_t max = n*n-1;
    const int64_t num_vars = (n*(n-1))/2;

    std::vector<IntVar *> Y;
    s.MakeIntVarArray(num_vars, 1, max, "Y_", &Y);

    s.AddConstraint(s.MakeAllDifferent(Y));

    int index = n-2;
    IntVar * v2 = NULL;
    for (int i=2; i<=n-1; i++) {
        for (int j=0; j<n-i; j++) {
            index++;
            v2 = Y[j];
            for (int p=j+1; p<=j+i-1; p++) {
                v2 = s.MakeSum(Y[p], v2)->Var();
            }
            s.AddConstraint(s.MakeEquality(Y[index], v2));
        }
    }

    OptimizeVar* const length = s.MakeMinimize(Y[num_vars-1], 1);
    SolutionCollector* const collector = s.MakeLastSolutionCollector();
    collector->Add(Y);

    DecisionBuilder* const db = s.MakePhase(Y, Solver::CHOOSE_FIRST_UNBOUND, Solver::ASSIGN_MIN_VALUE);

    s.Solve(db, collector, length);
    const int64_t result = collector->Value(0, Y[num_vars-1]);

    int64_t tick = 0;
    std::ostringstream oss;
    oss << "Solution: ";
    for (int i=0; i<=n-1; i++) {
        oss << tick << " ";
        tick += collector->Value(0, Y[i]);
    }
    LOG(INFO) << oss.str();
}

} // namespace operations_research

int main(int argc, char ** argv)
{
    // https://acrogenesis.com/or-tools/documentation/user_manual/manual/objectives/first_implementation.html
    for (int n=4; n<11; n++) {
        operations_research::Golomb(n);
    }
    return 0;
}