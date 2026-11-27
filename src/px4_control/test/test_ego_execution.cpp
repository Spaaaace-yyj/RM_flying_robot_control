#include "px4_control/ego_trajectory.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>

using namespace px4_control;
namespace
{
    int checks = 0;
    void check(bool value) { ++checks; if (!value) { throw std::runtime_error("check " + std::to_string(checks)); } }
    void near(const Vec3& a, const Vec3& b) { check((a - b).norm() < 1e-9); }
    template<class F> void throws(F f)
    {
        bool caught = false;
        try { f(); } catch (const std::exception&) { caught = true; }
        check(caught);
    }
    const Vec3 origin(0., 0., 1.), goal(1., 0., 1.), zero = Vec3::Zero();
    std::vector<Vec3> points()
    { return {origin, origin, origin, goal, goal, goal}; }
    std::vector<double> knots() { return {-3., -2., -1., 0., 1., 2., 3., 4., 5., 6.}; }
    EgoSpline spline() { return EgoSpline(3, points(), knots(), 2., 2.); }
    EgoExecution running()
    {
        EgoExecution e;
        check(e.begin(goal, origin, zero, 0., 100.).first);
        check(e.receive(spline(), 1, 100.1, origin, zero, 100.1).first);
        return e;
    }
    Observation flying()
    {
        Observation o;
        o.healthy = o.status_fresh = o.armed = true;
        o.nav_state = kOffboard;
        o.position = origin;
        return o;
    }
}

int main()
{
    try
    {
        // Rejection replies echo the exact forwarded request identity, not just a nearby point.
        check(goalRejectionMatches(100000001,goal,"odom",100000001,goal,"odom"));
        check(!goalRejectionMatches(100000002,goal,"odom",100000001,goal,"odom"));
        check(!goalRejectionMatches(100000001,goal,"odom",100000001,goal,"map"));
        check(!goalRejectionMatches(100000001,goal,"odom",100000001,origin,"odom"));
        check(!goalRejectionMatches(0,goal,"odom",0,goal,"odom"));
        check(!goalRejectionMatches(100000001,goal,"odom",100000001,Vec3(NAN,0,1),"odom"));
        EgoExecution newer;
        check(newer.begin(goal,origin,zero,0.,100.).first);
        if (goalRejectionMatches(100000002,goal,"odom",100000001,goal,"odom")) newer.cancel("stale rejection");
        check(newer.busy());
        if (goalRejectionMatches(100000002,goal,"odom",100000002,goal,"odom")) newer.cancel("current rejection");
        check(!newer.busy());
        // Independent polynomial oracles for position and both derivatives.
        const std::vector<double> k{-3., -2., -1., 0., 1., 2., 3., 4.};
        EgoSpline line(3, {Vec3(0,0,0), Vec3(1,0,0), Vec3(2,0,0), Vec3(3,0,0)}, k, 2., 2.);
        for (int i = 0; i <= 100; ++i)
        {
            const double t = i / 100.;
            auto r = line.sample(t);
            near(r.position, Vec3(1. + t, 0., 0.));
            near(r.velocity, Vec3(1., 0., 0.));
            near(r.acceleration, zero);
        }
        EgoSpline quadratic(3, {Vec3(0,0,0), Vec3(1,0,0), Vec3(4,0,0), Vec3(9,0,0)}, k, 6., 3.);
        for (double t : {0., .2, .7, 1.})
        {
            const auto r = quadratic.sample(t);
            near(r.position, Vec3(t*t + 2.*t + 4./3., 0., 0.));
            near(r.velocity, Vec3(2.*t + 2., 0., 0.));
            near(r.acceleration, Vec3(2., 0., 0.));
        }
        auto shifted = k;
        for (auto& t : shifted) { t += 17.; }
        EgoSpline translated_knots(3, {Vec3(0,0,0), Vec3(1,0,0), Vec3(2,0,0), Vec3(3,0,0)}, shifted, 2., 2.);
        near(translated_knots.sample(.25).position, line.sample(.25).position);
        near(spline().sample(0.).position, origin);
        near(spline().sample(3.).position, goal);
        near(spline().sample(0.).velocity, zero);
        near(spline().sample(3.).velocity, zero);
        near(spline().sample(1.5).position, Vec3(.5, 0., 1.));
        near(spline().sample(1.5).velocity, Vec3(.75, 0., 0.));
        throws([] { EgoSpline(2, points(), knots(), 2., 2.); });
        throws([] { EgoSpline(3, {}, {}, 2., 2.); });
        throws([] { auto p = points(); p[2].x() = NAN; EgoSpline(3,p,knots(),2.,2.); });
        throws([] { auto k = knots(); k[3] = NAN; EgoSpline(3,points(),k,2.,2.); });
        throws([] { auto k = knots(); k[3] = k[2]; EgoSpline(3,points(),k,2.,2.); });
        throws([] { auto k = knots(); k.pop_back(); EgoSpline(3,points(),k,2.,2.); });
        throws([] { EgoSpline(3,points(),knots(),.5,2.); });
        throws([] { EgoSpline(3,points(),knots(),2.,.5); });
        throws([] { spline().sample(NAN); });
        throws([] { EgoConfig c; c.min_z = c.max_z; c.validate(); });
        throws([] { EgoConfig c; c.planning_timeout = NAN; c.validate(); });
        throws([] { EgoConfig c; c.nearby_goal_radius = NAN; c.validate(); });
        throws([] { EgoConfig c; c.nearby_goal_radius = -.1; c.validate(); });
        throws([] { EgoConfig c; c.nearby_goal_radius = 2.1; c.validate(); });

        // Repeating an already reached goal succeeds as a hold without authorizing a spline.
        EgoExecution close;
        check(close.begin(origin,origin,zero,0.,100.).first && !close.busy());
        check(close.begin(origin+Vec3(.1,0,0),origin,zero,0.,100.).first && !close.busy());
        check(!close.receive(spline(),1,100.,origin,zero,100.).first);
        check(!close.begin(origin,origin,Vec3(.36,0,0),0.,100.).first);
        check(close.begin(origin+Vec3(.16,0,0),origin,zero,0.,100.).first && close.busy());
        close.cancel("test");
        check(close.begin(Vec3(4,0,1),origin,zero,0.,100.).second.find("distance_m=")!=std::string::npos);
        check(close.begin(Vec3(1,0,0),origin,zero,0.,100.).second.find("z_m=")!=std::string::npos);
        check(close.begin(goal,origin,Vec3(.36,0,0),0.,100.).second.find("speed_mps=")!=std::string::npos);

        // First valid complete spline chooses a nearby stop. Arrival uses this point,
        // while rejection correlation keeps the original request identity.
        const Vec3 nearby(.75,0,1);
        auto nearby_points=points(); for(int i=3;i<6;++i) nearby_points[i]=nearby;
        EgoSpline near_spline(3,nearby_points,knots(),2.,2.);
        EgoExecution adjusted;
        check(adjusted.begin(goal,origin,zero,0.,100.).first);
        check(adjusted.receive(near_spline,1,100.1,origin,zero,100.1).first);
        near(*adjusted.goal(),nearby); near(*adjusted.requestedGoal(),goal);
        check(goalRejectionMatches(100000001,*adjusted.requestedGoal(),"odom",100000001,goal,"odom"));
        auto frozen=adjusted;
        auto drift_points=points(); for(int i=3;i<6;++i) drift_points[i]=Vec3(.5,0,1);
        check(!frozen.receive(EgoSpline(3,drift_points,knots(),2.,2.),2,100.2,origin,zero,100.2).first);
        check(!frozen.busy()); // A later endpoint cannot wander within the original region.
        check(!adjusted.tick(3.2,103.2,nearby,zero,true,false));
        check(!adjusted.busy() && adjusted.reason().find("nearby goal reached")!=std::string::npos);
        for(const Vec3& delta : {Vec3(.51,0,0),Vec3(0,0,.16),Vec3(.4,.4,0)})
        {
            auto p=points(); for(int i=3;i<6;++i) p[i]=goal+delta;
            EgoExecution outside; check(outside.begin(goal,origin,zero,0.,100.).first);
            check(!outside.receive(EgoSpline(3,p,knots(),2.,2.),1,100.1,origin,zero,100.1).first);
            check(outside.busy() && !outside.tracking());
        }
        EgoConfig exact_config; exact_config.nearby_goal_radius=0.;
        EgoExecution exact(exact_config); check(exact.begin(goal,origin,zero,0.,100.).first);
        check(!exact.receive(near_spline,1,100.1,origin,zero,100.1).first);

        EgoExecution e;
        check(!e.receive(spline(), 1, 100., origin, zero, 100.).first);
        check(!e.begin(Vec3(4.,0.,1.), origin, zero, 0.,100.).first);
        check(!e.begin(Vec3(1.,0.,0.), origin, zero, 0.,100.).first);
        check(!e.begin(goal, origin, Vec3(1.,0.,0.), 0.,100.).first);
        check(e.begin(goal,origin,zero,0.,100.).first);
        check(!e.begin(goal,origin,zero,0.,100.).first);
        check(!e.receive(spline(),1,99.9,origin,zero,100.).first);
        check(!e.receive(spline(),1,101.,origin,zero,100.).first);
        check(!e.receive(spline(),1,100.,origin,zero,101.).first);
        check(e.busy() && !e.tracking());
        check(e.receive(spline(),1,100.1,origin,zero,100.1).first);
        check(!e.receive(spline(),1,100.2,origin,zero,100.2).first);
        check(e.tracking());
        // A new goal lease rejects a delayed spline from a cancelled goal.
        e.cancel("operator hold");
        check(e.begin(goal,origin,zero,1.,101.).first);
        check(!e.receive(spline(),2,100.9,origin,zero,101.).first);
        check(e.busy() && !e.tracking());
        auto far_points = points();
        for (auto& p : far_points) { p.x() += 1.; }
        EgoSpline wrong_goal(3, far_points, knots(), 2.,2.);
        check(!e.receive(wrong_goal,3,101.,origin,zero,101.).first);
        check(e.busy() && !e.tracking());
        // No p=0 fallback for a future spline; sample is bounded at its actual start.
        auto future = running();
        auto f = future.tick(.09,100.09,origin,zero,true,true);
        check(f.has_value()); near(f->position, origin);
        // Position jump, emergency spline, unsafe altitude all terminate the lease.
        auto jump = running();
        check(!jump.receive(spline(),2,100.2,Vec3(3.,0.,1.),zero,100.2).first);
        check(!jump.busy());
        auto stop = running();
        EgoSpline stationary(3, std::vector<Vec3>(6,origin),knots(),2.,2.);
        check(!stop.receive(stationary,2,100.2,origin,zero,100.2).first);
        check(!stop.busy());
        auto envelope = running();
        auto unsafe_points = points(); unsafe_points[2].z() = .29;
        EgoSpline unsafe(3,unsafe_points,knots(),10.,10.);
        check(!envelope.receive(unsafe,2,100.2,origin,zero,100.2).first);
        check(!envelope.busy());

        for (int failure = 0; failure < 5; ++failure)
        {
            auto task = running();
            const double ros = failure == 2 ? 99. : 100.2;
            const double steady = failure == 3 ? 46. : .2;
            const Vec3 p = failure == 4 ? Vec3(2.,0.,1.) : origin;
            check(!task.tick(steady,ros,p,zero,failure != 0,failure != 1));
            check(!task.busy());
        }
        EgoExecution pending;
        check(pending.begin(goal,origin,zero,0.,100.).first);
        check(!pending.tick(5.1,105.1,origin,zero,true,true));
        check(!pending.busy());
        // Elapsed spline time is not evidence of actual arrival.
        auto settle = running();
        auto end = settle.tick(3.2,103.2,Vec3(.8,0.,1.),zero,true,false);
        check(end.has_value() && settle.busy()); near(end->velocity,zero); near(end->acceleration,zero);
        check(!settle.tick(8.3,108.3,Vec3(.8,0.,1.),zero,true,false));
        check(!settle.busy());
        auto arrived = running();
        check(!arrived.tick(3.2,103.2,goal,zero,true,false));
        check(!arrived.busy());
        check(arrived.reason().find("goal reached") != std::string::npos);

        Alignment alignment;
        alignment.initialize(Vec3(5.,-2.,.2), Quat::Identity(), Quat(rotationZ(.7)), Vec3(10.,20.,-3.));
        const auto r = spline().sample(1.5);
        near(alignment.localPosition(r.position), alignment.convert(r.position,Quat::Identity(),r.velocity).position);
        near(alignment.localVector(r.velocity), alignment.convert(r.position,Quat::Identity(),r.velocity).velocity);
        near(alignment.worldPosition(alignment.localPosition(r.position)),r.position);
        near(alignment.worldVector(alignment.localVector(r.acceleration)),r.acceleration);
        near(alignment.localPosition(r.position + r.velocity) - alignment.localPosition(r.position),
             alignment.localVector(r.velocity));

        // Simulated 50 Hz closed-loop lifecycle through the existing Flight state machine.
        Flight flight;
        auto o = flying();
        flight.tick(0.,100.,o);
        check(flight.begin(0.,100.,o,false).first);
        flight.tick(1.6,101.6,o);
        check(flight.state == State::Hover);
        EgoExecution task;
        check(task.begin(goal,origin,zero,1.7,101.7).first);
        check(task.receive(spline(),1,101.8,origin,zero,101.8).first);
        for (int i = 0; i < 150; ++i)
        {
            const double t = i * .02;
            const auto ideal = spline().sample(t);
            o.position = ideal.position; o.velocity = ideal.velocity;
            auto reference = task.tick(1.8 + t,101.8 + t,o.position,o.velocity,true,true);
            check(reference.has_value());
            check(flight.track(reference->position,o).first);
            auto action = flight.tick(1.8 + t,101.8 + t,o);
            check(action.heartbeat && flight.state == State::TrackTrajectory);
            near(action.position,ideal.position);
        }
        o.position = goal; o.velocity = zero;
        check(!task.tick(4.9,104.9,goal,zero,true,false));
        check(flight.hold(4.9,o).first);
        check(flight.state == State::Hover);
        check(task.begin(origin,goal,zero,5.,105.).first); // Next explicit goal.
        check(flight.track(goal,o).first);
        o.nav_state = 2;
        check(!flight.tick(5.1,105.1,o).heartbeat);
        check(flight.state == State::Manual);
        check(!flight.track(goal,o).first);
        flight.state = State::TrackTrajectory;
        o.nav_state = kOffboard; o.healthy = false;
        check(!flight.tick(5.2,105.2,o).heartbeat);
        check(flight.state == State::Fault);
        std::cout << checks << " EGO execution checks passed\n";
    }
    catch (const std::exception& e)
    { std::cerr << e.what() << '\n'; return 1; }
}
