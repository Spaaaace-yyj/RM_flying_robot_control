#include "px4_control/ego_trajectory.hpp"
#include "bspline_opt/terminal_constraints.hpp"
#include <iostream>
#include <stdexcept>

using namespace px4_control;
namespace
{
int checks=0;
void check(bool ok) { ++checks; if (!ok) throw std::runtime_error("contract check "+std::to_string(checks)); }
void near(const Vec3& a,const Vec3& b) { check((a-b).norm()<1e-8); }
EgoSpline received(ego_planner::UniformBspline& curve)
{
    const auto cp=curve.getControlPoint(); const auto k=curve.getKnot();
    std::vector<Vec3> points; std::vector<double> knots;
    for(int i=0;i<cp.cols();++i) points.push_back(cp.col(i));
    for(int i=0;i<k.size();++i) knots.push_back(k(i));
    return EgoSpline(3,points,knots,100.,100.);
}
}
int main()
{
    try
    {
        const Vec3 origin(0.,0.,1.),zero=Vec3::Zero();
        for(int trial=0;trial<60;++trial)
        {
            const double h=.5+trial*.01;
            const Vec3 goal(.5+trial*.02,.1,1.);
            Eigen::MatrixXd points(3,10);
            const double fractions[]{0.,0.,0.,.1,.3,.6,.8,1.,1.,1.};
            for(int i=0;i<10;++i) points.col(i)=origin+fractions[i]*(goal-origin);
            ego_planner::constrainCubicTerminal(points,h,goal,zero,zero);
            ego_planner::UniformBspline actual(points,3,h);
            auto converted=received(actual);
            const double duration=actual.getTimeSum();
            check(std::abs(converted.duration()-duration)<1e-9);
            for(int i=0;i<=30;++i)
            {
                const double t=duration*i/30.; const auto sample=converted.sample(t);
                near(sample.position,actual.evaluateDeBoorT(t));
                near(sample.velocity,actual.getDerivative().evaluateDeBoorT(t));
                near(sample.acceleration,actual.getDerivative().getDerivative().evaluateDeBoorT(t));
            }
            EgoExecution task;
            check(task.begin(goal,origin,zero,0.,100.).first);
            check(task.receive(converted,1,100.1,origin,zero,100.1).first);
            check(task.tick(.1,100.1,origin,zero,true,true).has_value());
            check(!task.tick(duration+.2,100.2+duration,goal,zero,true,false));
            check(task.reason().find("goal reached")!=std::string::npos);
        }
        const Vec3 goal(1.,0.,1.);
        for(int failure=0;failure<2;++failure)
        {
            Eigen::MatrixXd points(3,10);
            for(int i=0;i<10;++i) points.col(i)=i<3?origin:goal;
            if(failure==0)
            {
                points.col(7)=goal-Vec3(.3,0.,0.);
                points.col(8)=goal-Vec3(.1,0.,0.);
                points.col(9)=goal+Vec3(.4,0.,0.); // Near goal, but nonzero endpoint speed.
            }
            else for(int i=7;i<10;++i) points.col(i)=goal+Vec3(.25,0.,0.);
            ego_planner::UniformBspline free_end(points,3,1.);
            EgoConfig strict; strict.nearby_goal_radius=0.;
            EgoExecution task(strict); check(task.begin(goal,origin,zero,0.,100.).first);
            const auto rejection=task.receive(received(free_end),1,100.1,origin,zero,100.1);
            check(!rejection.first);
            check(rejection.second.find("position_error_m=")!=std::string::npos);
            check(rejection.second.find("terminal_speed_mps=")!=std::string::npos);
        }
        // End-to-end contract: occupied request -> observed free candidate -> EGO's
        // constrained complete curve -> executor -> confirmed nearby arrival.
        auto free=[](const Vec3& p){return p.x()<=.7+1e-10;};
        const auto stop=ego_planner::nearbyFreeGoal(goal,origin,.5,.1,free);
        check(stop.has_value()); near(*stop,Vec3(.7,0,1));
        Eigen::MatrixXd candidate(3,10);
        const double progress[]{0.,0.,0.,.1,.3,.6,.8,1.,1.,1.};
        for(int i=0;i<10;++i) candidate.col(i)=origin+progress[i]*(*stop-origin);
        ego_planner::constrainCubicTerminal(candidate,1.,*stop,zero,zero);
        ego_planner::UniformBspline curve(candidate,3,1.);
        check(ego_planner::completeSplineClear(curve,.1,[&](const Vec3& p){return !free(p);}));
        EgoExecution nearby;
        check(nearby.begin(goal,origin,zero,0.,100.).first);
        check(nearby.receive(received(curve),1,100.1,origin,zero,100.1).first);
        near(*nearby.requestedGoal(),goal); near(*nearby.goal(),*stop);
        check(nearby.tick(.1,100.1,origin,zero,true,true).has_value());
        const double duration=curve.getTimeSum();
        check(!nearby.tick(duration+.2,100.2+duration,*stop,zero,true,false));
        check(nearby.reason().find("nearby goal reached")!=std::string::npos);
        std::cout<<checks<<" actual EGO/executor contract checks passed\n";
    }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
