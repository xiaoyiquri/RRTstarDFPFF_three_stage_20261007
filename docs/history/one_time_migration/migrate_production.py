from pathlib import Path
import hashlib,json,re,shutil
ROOT=Path(__file__).resolve().parents[1]
assert ROOT.name=='RRTstarDFPFF_paper_r2_float32_strict_20261007'
source=ROOT.parent/'RRTstarDFPFF_paper_r2_noguidance_20261006'
files=[p for p in source.rglob('*') if p.is_file() and not any(x.startswith('build') or x in ('.git','results','validation_logs','__pycache__') for x in p.relative_to(source).parts)]
snapshot=dict(original_root=str(source),copy_root=str(ROOT),excluded=['build*','.git','results','validation_logs','__pycache__'],files=[dict(path=p.relative_to(source).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files])
(ROOT/'docs/approved_source_snapshot.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2),encoding='utf-8')
history=ROOT/'docs/history/r2_baseline_docs';history.mkdir(exist_ok=True)
for name in ['README.md','运行指令.md','交付说明.md','修改说明.md','summary.md']:
    shutil.copy2(ROOT/name,history/name)
def edit(name, changes):
    p=ROOT/name;t=p.read_text(encoding='utf-8')
    for a,b in changes:
        if a not in t:
            if b in t: continue
            raise RuntimeError(f'{name}: missing {a[:80]}')
        t=t.replace(a,b)
    p.write_text(t,encoding='utf-8')
edit('common/flat_state.h',[
 ('template<int Dim, int Order>','template<int Dim, int Order, class Real = Scalar>'),
 ('Eigen::Matrix<Scalar,','Eigen::Matrix<Real,')])
edit('common/flat_control.h',[
 ('template<int Dim>','template<int Dim, class Real = Scalar>'),
 ('Eigen::Matrix<Scalar,','Eigen::Matrix<Real,'),
 ('Scalar& operator','Real& operator')])
edit('common/flat_trajectory.h',[
 ('typename Metadata = std::monostate>','typename Metadata = std::monostate, class Real = Scalar>'),
 ('FlatState<Dim,Order>','FlatState<Dim,Order,Real>'),('FlatControl<Dim>','FlatControl<Dim,Real>'),
 ('double T{0}, cost{0};','Real T{0}, cost{0};\n    bool normalized{false};'),
 ('Eigen::Matrix<double,','Eigen::Matrix<Real,'),
 ('derivative(double t','derivative(Real t'),('evaluate(double t','evaluate(Real t'),
 ('evaluateState(double t','evaluateState(Real t'),('evaluateControl(double t','evaluateControl(Real t'),
 ('control(double t','control(Real t'),('velocity(double t','velocity(Real t'),
 ('acceleration(double t','acceleration(Real t'),('jerk(double t','jerk(Real t'),('snap(double t','snap(Real t'),
 ('std::array<double,Width>','std::array<Real,Width>'),
 ('t < -1e-10||t > T+1e-10','t < -Real(1e-6)||t > T+Real(1e-6)'),
 ('t=std::clamp(t,0.0,T);','t=std::clamp(t,Real(0),T);\n        if (normalized) t /= T;'),
 ('result=result*t+coefficients.col(k)*(factorial(k)/factorial(k-d));','{\n            Real factor = Real(1);\n            for (int j=0;j<d;++j) factor *= Real(k-j);\n            result=result*t+coefficients.col(k)*factor;\n        }\n        if (normalized) for(int j=0;j<d;++j) result /= T;')])
edit('core/node.h',[
 ('typename MetadataT = std::monostate>','typename MetadataT = std::monostate, class Real = Scalar>'),
 ('FlatState<Dim, Order>','FlatState<Dim, Order, Real>'),('FlatTrajectory<Dim, Order, MetadataT>','FlatTrajectory<Dim, Order, MetadataT, Real>'),
 ('double cost{0};','Real cost{0};')])
edit('common/planner_config.h',[
 ('struct PlannerConfig {','template<class Real = double> struct PlannerConfigT {'),
 ('double timeout{10}, goal_bias{.1}, range{4}, rewire_factor{3};','double timeout{10};\n    Real goal_bias{Real(.1)}, range{4}, rewire_factor{3};'),
 ('double goal_tolerance{.05};','Real goal_tolerance{Real(.05)};'),
 ('unsigned seed{40};','std::size_t hard_safety_node_cap{1000000};\n    unsigned seed{40};'),
 ('goal_tolerance < 0 || !max_iterations || max_nodes < 2 || !goal_parent_limit ||','goal_tolerance < 0 || (max_nodes && max_nodes < 2) || !goal_parent_limit ||\n            (hard_safety_node_cap && hard_safety_node_cap < 2) ||'),
 ('} // namespace rrtpff','using PlannerConfig = PlannerConfigT<double>;\n} // namespace rrtpff')])
edit('core/sampler.h',[
 ('using Output = typename Model::FlatOutput;','using Output = typename Model::FlatOutput;\n    using Real = typename Output::Scalar;'),
 ('sampleFlatOutput(double goal_bias','sampleFlatOutput(Real goal_bias'),
 ('std::uniform_real_distribution<double>','std::uniform_real_distribution<Real>')])
edit('core/rewiring.h',[
 ('public:\n','public:\n    using Real = decltype(Node::cost);\n',),
 ('static double propagateCostToChildren','static Real propagateCostToChildren'),
 ('double largest_change','Real largest_change'),('const double cost','const Real cost'),
 ('1e-7','Real(1e-5)'),('1e-10','Real(1e-7)')])
edit('core/nearest_neighbor.h',[
 ('using Output = typename Model::FlatOutput;','using Output = typename Model::FlatOutput;\n    using Real = typename Output::Scalar;'),
 ('double','Real'),
 ('std::numeric_limits<Real>::epsilon()','std::numeric_limits<Real>::epsilon()'),
 ('const Real tol = 64 *','const Real tol = Real(64) *'),
 ('(1 +','(Real(1) +'),
 ('tree_.size() >= 4','tree_.size() >= (std::is_same_v<Real,float> ? 8 : 4)'),
 ('distanceBatch(coordinates_.data(), query.data(), weights.data(), periods.data(),','distanceBatchTyped(coordinates_.data(), query.data(), weights.data(), periods.data(),'),
 ('vector_evaluations_ += (nodes_.size() / 4) * 4;','constexpr std::size_t W = std::is_same_v<Real,float> ? 8 : 4;\n        vector_evaluations_ += (nodes_.size() / W) * W;')])
edit('core/rrtstar.h',[
 ('template <class SystemModelT> class RRTstar {',"""template<class M, class = void> struct PlannerSolverTypes {
    using Solver = PFFSolver<M::FlatDim,M::FlatOrder,typename M::Metadata>;
    using BatchSolver = SIMD_PFF<M::FlatDim,M::FlatOrder,typename M::Metadata,SIMDBatchWidth>;
};
template<class M> struct PlannerSolverTypes<M,std::void_t<typename M::Solver,typename M::BatchSolver>> {
    using Solver = typename M::Solver;
    using BatchSolver = typename M::BatchSolver;
};
template <class SystemModelT> class RRTstar {"""),
 ('using Node = RRTNode<Model::FlatDim, Model::FlatOrder, Metadata>;','using Real = typename Output::Scalar;\n    using Node = RRTNode<Model::FlatDim, Model::FlatOrder, Metadata, Real>;'),
 ('using Solver = PFFSolver<Model::FlatDim, Model::FlatOrder, Metadata>;','using Solver = typename PlannerSolverTypes<Model>::Solver;'),
 ('using BatchSolver = SIMD_PFF<Model::FlatDim, Model::FlatOrder, Metadata, BatchWidth>;','using BatchSolver = typename PlannerSolverTypes<Model>::BatchSolver;'),
 ('RRTstar(Model model, PlannerConfig config','RRTstar(Model model, PlannerConfigT<Real> config'),
 ('PlannerConfig config_;','PlannerConfigT<Real> config_;'),
 ('path_.clear();','path_.clear();\n        first_path_.clear();'),
 ('while (wall.seconds() < config_.timeout && stats_.iterations < config_.max_iterations &&\n               tree_.size() < config_.max_nodes)',"""while (wall.seconds() < config_.timeout &&
               (!config_.max_iterations || stats_.iterations < config_.max_iterations) &&
               (!config_.max_nodes || tree_.size() < config_.max_nodes))"""),
 ('++stats_.iterations;','if (config_.hard_safety_node_cap && tree_.size() >= config_.hard_safety_node_cap) {\n                stats_.termination_reason = "safety_limit";\n                stats_.completed_normally = false;\n                break;\n            }\n            ++stats_.iterations;'),
 ('const double n = static_cast<double>(tree_.size() + 1);','const Real n = static_cast<Real>(tree_.size() + 1);'),
 ('const double radius =','const Real radius ='),
 ('1.0 / Model::FlatDim','Real(1) / Real(Model::FlatDim)'),
 ("""if (!std::isfinite(stats_.first_solution_time))
                    stats_.first_solution_time = wall.seconds();""","""if (!std::isfinite(stats_.first_solution_time)) {
                    stats_.first_solution_time = wall.seconds();
                    stats_.first_solution_cost = double(added->cost);
                    for (Node *n=added;n->parent;n=n->parent) first_path_.push_back(n->incoming);
                    std::reverse(first_path_.begin(),first_path_.end());
                }
                if (config_.stop_at_first_solution) {
                    stats_.termination_reason = "first_solution";
                    break;
                }"""),
 ('Node *best = bestGoal();\n        stats_.node_count',"""if (stats_.termination_reason.empty()) {
            if (config_.max_iterations && stats_.iterations >= config_.max_iterations)
                stats_.termination_reason = "max_iterations";
            else if (config_.max_nodes && tree_.size() >= config_.max_nodes)
                stats_.termination_reason = "max_nodes";
            else stats_.termination_reason = "timeout";
        }
        Node *best = bestGoal();
        stats_.node_count"""),
 ('model_.distanceToOutput(node->state, model_.goalOutput()));','double(model_.distanceToOutput(node->state, model_.goalOutput())));'),
 ('if (stats_.iterations >= config_.max_iterations)','if (config_.max_iterations && stats_.iterations >= config_.max_iterations)'),
 ('else if (tree_.size() >= config_.max_nodes)','else if (config_.max_nodes && tree_.size() >= config_.max_nodes)'),
 ('stats_.failure_reason = "timeout_no_goal";','stats_.failure_reason = stats_.termination_reason == "safety_limit" ? "safety_limit" : "timeout_no_goal";'),
 ('auto previous = edge.evaluate(0).output();','auto previous = edge.evaluate(Real(0)).output();'),
 ('edge.T * i / 100','edge.T * Real(i) / Real(100)'),
 ('double connected_parent_cost{std::numeric_limits<double>::infinity()};','Real connected_parent_cost{std::numeric_limits<Real>::infinity()};'),
 ('double total;','Real total;'),('const double total = parent','const Real total = parent'),
 ('1e-10','Real(1e-7)'),
 ('std::vector<Trajectory> path_;','std::vector<Trajectory> path_, first_path_;'),
 ('const RRTTree<Node> &tree() const { return tree_; }','const RRTTree<Node> &tree() const { return tree_; }\n    const std::vector<Trajectory> &firstPath() const { return first_path_; }')])
# Record anytime cost improvements outside all cost comparisons.
p=ROOT/'core/rrtstar.h';t=p.read_text()
t=t.replace('if (config_.prune)\n                pruneTree();',"""if (config_.prune)
                pruneTree();
            if (Node *g=bestGoal()) {
                if (stats_.best_cost_events.empty() || double(g->cost) < stats_.best_cost_events.back().second)
                    stats_.best_cost_events.emplace_back(wall.seconds(),double(g->cost));
            }""")
t=t.replace('stats_.planning_time = wall.seconds();\n        for (const auto &node', 'stats_.planning_time = wall.seconds();\n        if (best && stats_.best_cost_events.empty()) stats_.best_cost_events.emplace_back(stats_.first_solution_time,double(best->cost));\n        for (const auto &node')
p.write_text(t,encoding='utf-8')
edit('common/types.h',[
 ('struct PlanResult {','''struct PlanResult {
    std::string numeric_profile{"legacy_mixed"}, solver_precision{"float64"}, time_root_precision{"long_double"};
    std::string termination_reason;
    bool completed_normally{true};
    double first_solution_cost{std::numeric_limits<double>::infinity()};
    std::vector<std::pair<double,double>> best_cost_events;
    std::size_t speed_certificate_rejected{}, speed_certificate_uncertain{};
''')])
edit('simd/simd_backend.h',[
 ('bool simdCompiledAVX2();','''void distanceBatchFloat(const float*, const float*, const float*, const float*, int,
                        std::size_t, std::size_t, float*, SIMDBackend);
bool simdCompiledAVX2();''')]) if False else None
# Place typed NN overloads after the original SIMDBackend/ distanceBatch declarations.
p=ROOT/'simd/simd_backend.h';t=p.read_text();a='\n} // namespace rrtpff\n';i=t.index(a)
t=t[:i]+'''
void distanceBatchFloat(const float*, const float*, const float*, const float*, int,
                        std::size_t, std::size_t, float*, SIMDBackend);
inline void distanceBatchTyped(const float* a,const float* b,const float* c,const float* d,int n,
                               std::size_t count,std::size_t stride,float* out,SIMDBackend backend) {
    distanceBatchFloat(a,b,c,d,n,count,stride,out,backend);
}
inline void distanceBatchTyped(const double* a,const double* b,const double* c,const double* d,int n,
                               std::size_t count,std::size_t stride,double* out,SIMDBackend backend) {
    distanceBatch(a,b,c,d,n,count,stride,out,backend);
}
'''+t[i:];p.write_text(t,encoding='utf-8')
# Native spheres need center-rounding compensation as well as boxes.
edit('dynobench/data_adapter.cpp',[
 ('s.radius = float(positive(obstacle, "radius"));','double error2 = 0;\n            for (int k=0;k<pdim;++k) error2 += std::pow(center(k)-double(s.center[k]),2);\n            s.radius = std::nextafter(float(positive(obstacle,"radius") + std::sqrt(error2)),std::numeric_limits<float>::infinity());')])
# Saved diagnostics are permitted double reporting; the legacy cost semantics stay labelled.
