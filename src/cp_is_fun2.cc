#include "ortools/constraint_solver/constraint_solver.h"
#include "ortools/base/logging.h"

#include <vector>

namespace operations_research  {

IntVar* const MakeBaseLine2(Solver* s, IntVar * const v1, IntVar * const v2, const int64_t base)
{
    // (v1 * base) + v2
    return s->MakeSum(s->MakeProd(v1, base), v2)->Var();
}

IntVar* const MakeBaseLine3(Solver* s, IntVar * const v1, IntVar * const v2, IntVar * const v3, const int64_t base)
{
    // (v1*base*base) + (v2*base) + (v3)
    IntVar * const rVar = MakeBaseLine2(s, v2, v3, base);
    return s->MakeSum(s->MakeProd(v1, base*base), rVar)->Var();

    // std::vector<IntVar*> vars;
    // std::vector<int64_t> coefficients;
    // vars.push_back(v1);
    // coefficients.push_back(base * base);
    // vars.push_back(v2);
    // coefficients.push_back(base);
    // vars.push_back(v3);
    // coefficients.push_back(1);
    // 
    // return s->MakeScalProd(vars, coefficients)->Var();
}

IntVar* const MakeBaseLine4(Solver* s, IntVar * const v1, IntVar * const v2, IntVar * const v3, IntVar * const v4, const int64_t base)
{
    IntVar * const rVar = MakeBaseLine3(s, v2, v3, v4, base);
    return s->MakeSum(s->MakeProd(v1, base*base*base), rVar)->Var();

    // std::vector<IntVar*> vars;
    // std::vector<int64_t> coefficients;
    // vars.push_back(v1);
    // coefficients.push_back(base * base * base);
    // vars.push_back(v2);
    // coefficients.push_back(base * base);
    // vars.push_back(v3);
    // coefficients.push_back(base);
    // vars.push_back(v4);
    // coefficients.push_back(1);
    // 
    // return s->MakeScalProd(vars, coefficients)->Var();
}

void CPIsFun()
{
    Solver solver("cp is fun");
    const int64_t kBase = 10;
    //     C P
    //     I S
    //   F U N
    // T R U E
    //
    // U 는 같은 자리 수 이므로 변수는 하나만 있으면 된다.
    IntVar * const c = solver.MakeIntVar(1, kBase-1, "C");
    IntVar * const p = solver.MakeIntVar(0, kBase-1, "P");

    IntVar * const i = solver.MakeIntVar(1, kBase-1, "I");
    IntVar * const s = solver.MakeIntVar(0, kBase-1, "S");
    
    IntVar * const f = solver.MakeIntVar(1, kBase-1, "F");
    IntVar * const u = solver.MakeIntVar(0, kBase-1, "U");
    IntVar * const n = solver.MakeIntVar(0, kBase-1, "N");
    
    IntVar * const t = solver.MakeIntVar(1, kBase-1, "T");
    IntVar * const r = solver.MakeIntVar(0, kBase-1, "R");
    IntVar * const e = solver.MakeIntVar(0, kBase-1, "E");

    std::vector<IntVar*> letters{c,p,i,s,f,u,n,t,r,e};

    // Constraint
    // All different
    solver.AddConstraint(solver.MakeAllDifferent(letters));

    IntVar * const term1 = MakeBaseLine2(&solver, c, p, kBase);
    IntVar * const term2 = MakeBaseLine2(&solver, i, s, kBase);
    IntVar * const term3 = MakeBaseLine3(&solver, f, u, n, kBase);
    IntVar * const sum_terms = solver.MakeSum(solver.MakeSum(term1, term2), term3)->Var();

    IntVar * const sum = MakeBaseLine4(&solver, t, r, u, e, kBase);

    // Constraint
    // CP + IS + FUN == TRUE
    solver.AddConstraint(solver.MakeEquality(sum_terms, sum));

    SolutionCollector* const all_solutions = solver.MakeAllSolutionCollector();
    // Add the interesting variables to the SolutionCollector
    all_solutions->Add(c);
    all_solutions->Add(p);
    // Create the variable kBase * c + p
    IntVar* v1 = solver.MakeSum(solver.MakeProd(c, kBase), p)->Var();
    // Add it to the SolutionCollector
    all_solutions->Add(v1);
    // all_solutions->Add(letters);

    // Search tree
    DecisionBuilder * const db = solver.MakePhase(letters, Solver::CHOOSE_FIRST_UNBOUND, Solver::ASSIGN_MIN_VALUE);
    solver.Solve(db, all_solutions);

    const int nrSolutions = all_solutions->solution_count();
    LOG(INFO) << "Number of solutions: " << nrSolutions << '\n';
    for (int i=0; i<nrSolutions; i++) {
        Assignment* const solution = all_solutions->solution(i);
        LOG(INFO) << "Solution found:";
        LOG(INFO) << "v1=" << solution->Value(v1);
    }
}

} // namespace operations_research

int main()
{
    // https://acrogenesis.com/or-tools/documentation/user_manual/manual/first_steps/monitors.html
    operations_research::CPIsFun();
    return 0;
}
