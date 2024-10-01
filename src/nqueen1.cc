#include "ortools/base/commandlineflags.h"
#include "ortools/base/logging.h"
#include "ortools/constraint_solver/constraint_solver.h"

#include <vector>

namespace operations_research {

void PrintSolution(const int size, // board size
                   const std::vector<IntVar*>& queens,
                   SolutionCollector* const collector,
                   const int solution_number) 
{
    if (solution_number < collector->solution_count() && size < 100) {
        for (int j=0; j<size; j++) {
            for (int i=0; i<size; i++) {
                const int pos = collector->Value(solution_number, queens[i]);
                std::cout << std::setw(2);
                if (pos == j) {
                    std::cout << i;
                }
                else {
                    std::cout << ".";
                }
                std::cout << " ";
            }
            std::cout << "\n";
        }
    }
}

void PrintAllSolution(const int size, 
                      const std::vector<IntVar*>& queens,
                      SolutionCollector* const collector)
{
    for (int sol=0; sol<collector->solution_count(); sol++) {
        std::cout << "============================" << std::endl;
        PrintSolution(size, queens, collector, sol);
    }
}

void NQueens(int size)
{
    Solver s("nqueens");

    std::vector<IntVar*> queens;
    for (int i=0; i<size; i++) {
        queens.push_back(s.MakeIntVar(0, size-1, std::to_string(i)));
    }
    s.AddConstraint(s.MakeAllDifferent(queens));

    std::vector<IntVar*> vars(size);
    for (int i=0; i<size; i++) {
        vars[i] = s.MakeSum(queens[i], i)->Var();
    }
    s.AddConstraint(s.MakeAllDifferent(vars));

    for (int i=0; i<size; i++) {
        vars[i] = s.MakeSum(queens[i], -i)->Var();
    }
    s.AddConstraint(s.MakeAllDifferent(vars));

    SolutionCollector* const solution_counter = s.MakeAllSolutionCollector(NULL);
    SolutionCollector* const collector = s.MakeFirstSolutionCollector(); // only save first
    SolutionCollector* const mega_collector = s.MakeAllSolutionCollector();

    collector->Add(queens);
    mega_collector->Add(queens);

    std::vector<SearchMonitor*> monitors; 
    monitors.push_back(solution_counter);
    monitors.push_back(collector);
    monitors.push_back(mega_collector);

    DecisionBuilder* const db = s.MakePhase(queens, Solver::CHOOSE_FIRST_UNBOUND, Solver::ASSIGN_MIN_VALUE);

    s.Solve(db, monitors);
    const int num_solutions = solution_counter->solution_count();

    const int64_t time = s.wall_time();
    std::cout << "============================" << std::endl;
    std::cout << "size: " << size << std::endl;
    std::cout << "The Solve method took " << time/1000.0 << " seconds" << std::endl;
    std::cout << "number of solutions: " << num_solutions << std::endl;
    std::cout << "collector solution count: " << collector->solution_count() << "\n";
    std::cout << "mega_collector solution count: " << mega_collector->solution_count() << "\n";

    PrintSolution(size, queens, collector, 0);
    PrintAllSolution(size, queens, mega_collector);
}
} // namespace operations_research

int main()
{
    operations_research::NQueens(4);
    return 0;
}