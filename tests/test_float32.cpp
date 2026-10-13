#include "tests/test_support.h"
#include "pff/r2_float_kernel.h"
#define RRTPFF_DOUBLE_REFERENCE_TEST_ONLY 1
#include "tests/double_reference.h"
#include "production/unicycle_speed_certificate.h"
#include "production/float_validator.h"
#include "core/node.h"
#include "dynobench/conservative_sphere.h"
#include "postprocess/shortcut.h"
#include <type_traits>
using namespace rrtpff;
using namespace rrtpff::production;
struct ShortcutModel {
    using FlatOutput=Eigen::Vector2f;
    using FlatTrajectory=rrtpff::FlatTrajectory<2,2,std::monostate,float>;
    bool reject{false};
    bool prepareEdge(const FlatState<2,2,float>&,std::monostate,FlatTrajectory&)const{return true;}
    std::monostate terminalMetadata(const FlatTrajectory&)const{return {};}
    bool terminalMetadataMatches(const FlatTrajectory&,std::monostate)const{return true;}
    bool trajectoryValid(const FlatTrajectory&,PlanResult&)const{return !reject;}
};
int main(){return runTest([]{
    using S=R2FloatSolver<2>;using Ref=reference::DoubleR2<2>;
    static_assert(std::is_same_v<S::State::Vector::Scalar,float>);
    static_assert(std::is_same_v<decltype(RRTNode<2,2,std::monostate,float>{}.cost),float>);
    for(double scale:{1.,100000.,1e12})for(int dim:{2,3}){
        const std::array<double,3> center{{.123456789*scale,-.987654321*scale,.333333333*scale}};
        const double radius=.08660254037844386;
        const auto sphere=conservativeSphere(center,radius,dim);
        long double error=0;for(int k=0;k<dim;++k){
            const long double d=(long double)center[k]-sphere.center[k];error+=d*d;
        }
        require((long double)sphere.radius>=radius+std::sqrt(error),"float sphere encloses reference sphere and center error");
    }
    S::Weight R;R<<1.3f,.2f,.2f,.8f;
    FloatPFFConfig fc;fc.min_time=.1f;fc.max_time=6;fc.additional_time_samples=12;
    S solver(R,fc);PFFConfig rc;rc.min_time=fc.min_time;rc.max_time=fc.max_time;
    Ref reference(R.cast<double>(),rc);
    std::array<S::State,8> starts;std::array<S::Constraint,8> targets;
    for(int i=0;i<40;++i){
        S::State x,xf;x.values<<.13f*i,-.07f*i,.2f+.003f*i,-.1f;
        xf.values<<.8f+.02f*i,.6f,.3f,-.02f*i;
        Ref::State xd,xfd;xd.values=x.values.cast<double>();xfd.values=xf.values.cast<double>();
        for(bool fixed:{false,true}){
            const auto c=fixed?S::Constraint::full(xf):S::Constraint::output(xf.output());
            const auto cd=fixed?Ref::Constraint::full(xfd):Ref::Constraint::output(xfd.output());
            const float T=.3f+.09f*i;
            auto f=solver.fixedTime(x,c,T);auto d=reference.fixedTime(xd,cd,T);
            require(f.success&&d.success,"both boundary kernels produce valid solutions");
            close(f.cost,d.cost,3e-5,"independent double-reference cost");
            require((f.terminal_state.values.cast<double>()-d.terminal_state.values).norm()<3e-4,
                    "free terminal velocity and fixed state match double reference");
            for(int k=0;k<4;++k){
                const float t=T*float(k)/3.f;
                for(int derivative=0;derivative<4;++derivative)
                    require((f.trajectory.derivative(t,derivative).cast<double>()-
                             d.trajectory.derivative(t,derivative)).norm()<.003,
                            "normalized cubic physical derivatives match reference");
                require(f.trajectory.snap(t).isZero(),"snap zero exactly");
            }
            auto optimum=solver.solve(x,c);auto optimum_d=reference.solve(xd,cd);
            require(optimum.success&&optimum_d.success,"free-time candidates exist");
            close(optimum.cost,optimum_d.cost,3e-4,"minimum among roots and boundaries");
            const auto list=solver.candidates(x,c);
            require(std::any_of(list.begin(),list.end(),[&](auto v){return v.T==fc.min_time;}),"minimum time compared");
            require(std::any_of(list.begin(),list.end(),[&](auto v){return v.T==fc.max_time;}),"maximum time compared");
        }
        if(i<8){starts[i]=x;targets[i]=i%2?S::Constraint::full(xf):S::Constraint::output(xf.output());}
    }
    if(simdRuntimeAVX2()){
        R2FloatBatch<2> batch(solver);std::size_t lanes=0;
        auto simd=batch.candidatesBatch(starts,targets,7,true,&lanes);
        require(lanes==7,"mixed boundaries and partial AVX batch");
        for(int i=0;i<7;++i){auto scalar=solver.candidates(starts[i],targets[i]);
            close(simd[i].front().cost,scalar.front().cost,5e-5,"AVX float cost ranking parity");}
        float coordinates[26],query[2]{.7f,.3f},weights[2]{1.2f,.6f},periods[2]{0,0},a[13],b[13];
        for(int i=0;i<26;++i)coordinates[i]=float(i)*.1f;
        distanceBatchFloat(coordinates,query,weights,periods,2,13,13,a,SIMDBackend::Scalar);
        distanceBatchFloat(coordinates,query,weights,periods,2,13,13,b,SIMDBackend::AVX2);
        for(int i=0;i<13;++i)close(a[i],b[i],2e-6,"float NN with five scalar tail lanes");
    }
    std::array<float,5> repeated{{.0625f,-.5f,1.5f,-2.f,1.f}}; // (x-.5)^4
    auto roots=floatRoots(repeated,4,.1f,1.f);
    require(std::any_of(roots.begin(),roots.end(),[](float x){return std::abs(x-.5f)<.001f;}),"multiple root retained");
    S::State zero;auto same=solver.solve(zero,S::Constraint::full(zero));
    close(same.T,fc.min_time,1e-6,"degenerate zero-energy selects lower bound");
    ShortcutModel::FlatTrajectory tr;tr.T=1;tr.normalized=true;
    tr.coefficients(0,1)=.3f;
    require(certifyUnicycleSpeed(tr,1e-6f)==SpeedCertificate::Valid,"constant nonzero velocity certified");
    // v=(s-.371)^2: positive coarse samples, exact hidden interior stop.
    tr.coefficients(0,1)=.371f*.371f;tr.coefficients(0,2)=-.371f;tr.coefficients(0,3)=1.f/3.f;
    require(certifyUnicycleSpeed(tr,1e-5f)!=SpeedCertificate::Valid,"hidden stop cannot pass sample-only check");
    tr.coefficients.setZero();tr.coefficients(0,1)=1e-8f;
    require(certifyUnicycleSpeed(tr,1e-6f)==SpeedCertificate::ZeroOrBelowThreshold,"near-zero rejected");
    tr.coefficients(0,1)=.3f;
    require(certifyUnicycleSpeed(tr,1e-6f,20,0)==SpeedCertificate::Uncertain,"budget exhaustion fails conservatively");
    S::State a,b,c;a.values<<0,0,.3f,0;b.values<<.5f,0,.3f,0;c.values<<1,0,.3f,0;
    auto e1=solver.fixedTime(a,S::Constraint::full(b),.1f).trajectory;
    auto e2=solver.fixedTime(b,S::Constraint::full(c),.1f).trajectory;
    e1.cost=e2.cost=.01f;std::vector<ShortcutModel::FlatTrajectory> path{e1,e2};
    PlanResult counters;ShortcutConfig sc;ShortcutModel model;
    auto flask=shortcutPath(path,model,solver,sc,counters);
    require(flask.accepted==1&&flask.path.size()==1&&flask.path[0].cost>.02f,
            "flask accepts valid connection without cost decrease");
    require(path.size()==2,"raw path unchanged");
    sc.mode="cost_decreasing";
    require(shortcutPath(path,model,solver,sc,counters).accepted==0,"enhanced policy rejects cost increase");
    sc.mode="flask";model.reject=true;
    require(shortcutPath(path,model,solver,sc,counters).accepted==0,"invalid shortcut rejected");
    sc.enabled=false;
    require(shortcutPath(path,model,solver,sc,counters).seconds==0,"disabled shortcut timing zero");
});}
