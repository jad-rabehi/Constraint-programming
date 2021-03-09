#include <unistd.h> // used for usleep
#include <tubex.h>

using namespace std;
using namespace ibex;
using namespace tubex;


/* ------ -------- ------------- -------
 * ----------    MAIN    ---------------
 * ----------------------------------*/

int main()
{
    double dt = 0.05;
    double iteration_dt = 1; //0.2; //0.2; // elapsed animation time between each dt
    Interval tdomain(0,15); // [t0,tf]

    // Noises bounds
    double nu = 0.03;
    double nx3 = 0.03;

    RandTrajectory N_u(tdomain, dt, Interval(-nu,nu));        // The input noise signal
    RandTrajectory N_x3m(tdomain, dt, Interval(-nx3,nx3));    // The measurement "heading" noise signal

    // Initial pose x0=(0,0,2)
    Vector x0({0,0,2});

    // System input
    Trajectory u(tdomain, TFunction("3*(sin(t)^2)+t/100"), dt);    // Trajectory of u

    // Actual trajectories (state + derivative)
    TrajectoryVector v_truth(3);
    TrajectoryVector x_truth(3);

    v_truth[2] = u + N_u;
    x_truth[2] = v_truth[2].primitive() + x0[2];

    v_truth[0] = 10*cos(x_truth[2]+N_x3m);
    v_truth[1] = 10*sin(x_truth[2]+N_x3m);
    x_truth[0] = v_truth[0].primitive() + x0[0];
    x_truth[1] = v_truth[1].primitive() + x0[1];

    vibes::beginDrawing();
    VIBesFigMap figure("H: Online SLAM");
    figure.set_properties(50, 50, 600, 400);
    figure.add_trajectory(&x_truth, "x*", 0, 1);
    figure.show(0.5);


    /* -----  Creating and Plotting Tubes  ------- */

    // Tube of u
    Tube u_tube(tdomain,dt, TFunction("3*(sin(t)^2)+t/100"));

    // Actual tubes (stte + derivative)
    TubeVector x_tube(tdomain, dt, 3);
    TubeVector v_tube(tdomain, dt, 3);

    v_tube[2] = u_tube.inflate(nu);

    Tube x2(x_truth[2], dt);
    x_tube[2] = x2;
    x_tube[2].inflate(nx3);         // additional interval to enclose heading noise

    v_tube[0] = 10*cos(x_tube[2]+Interval(-nx3,nx3)); //
    v_tube[1] = 10*sin(x_tube[2]+Interval(-nx3,nx3)); // .inflate(nx3)
    x_tube[0] = v_tube[0].primitive() + x0[0];
    x_tube[1] = v_tube[1].primitive() + x0[1];

    figure.add_tube(&x_tube, "x", 0, 1);
    figure.smooth_tube_drawing(true);
    figure.show();

    /*  ------- BEACONS -------------*/

    vector<IntervalVector> M;
    M.push_back(IntervalVector({{6},{12}}));
    M.push_back(IntervalVector({{-2},{-5}}));
    M.push_back(IntervalVector({{-3},{20}}));
    M.push_back(IntervalVector({{3},{4}}));
/*
    M.push_back(IntervalVector({{12},{-12}}));
    M.push_back(IntervalVector({{-2},{1}}));
    M.push_back(IntervalVector({{20},{-15}}));
    M.push_back(IntervalVector({{3},{-10}}));
*/
    // unknown beacon associated for each observation
    int nb_beacons = M.size();
    vector<IntervalVector> UnknownBeacons(nb_beacons, IntervalVector(2));

    /*  ----- Define Contractors  -----*/
    CtcFunction ctc_distance(
                Function("x[2]", "bk[2]", "dk", "sqrt(sqr(x[0]-bk[0])+sqr(x[1]-bk[1]))-dk"));  // Contractor for distance
    CtcFunction ctc_systemEquation(
                Function("x[3]","u","v[3]","(10*cos(x[2])-v[0]; 10*sin(x[2])-v[1]; u[0]-v[2])"));
    CtcEval ctc_evaluation;
    CtcDeriv ctc_derivation;


    // Create Contractor Network
    ContractorNetwork cn;

    // Contractor for the system's equation
    cn.add(ctc_systemEquation, {x_tube, u_tube, v_tube});
    cn.add(ctc_derivation, {x_tube, v_tube});

    // ===========  ===== SLAM =======  =========
    int nb_obs = tdomain.ub()/(2*dt);
    vector<int> idx(nb_obs);
    vector<double> tt(nb_obs);


    //vector<Interval> v_obs;
    Interval distance;

    int i = 0;
    int j;
    double prev_t_obs = tdomain.lb();

    for(double t = tdomain.lb() ; t < tdomain.ub() ; t+=dt)
    {
        if(t - prev_t_obs > 10*dt) // new observation each 2*dt
        {
            // Creating new observation to a random landmark
            j = rand() % nb_beacons;
            idx[i] = j;
            tt[i] = t;

            //Interval& distance_ptr = cn.create_dom(sqrt(sqr(x_truth(t)[0]-M[j][0])+sqr(x_truth(t)[1]-M[j][1])));
            //distance_ptr = (sqrt(sqr(x_truth(t)[0]-M[j][0])+sqr(x_truth(t)[1]-M[j][1]))).inflate(0.03);

            distance = sqrt(sqr(x_truth(t)[0]-M[j][0])+sqr(x_truth(t)[1]-M[j][1]));
            distance.inflate(0.03);
            //v_obs.push_back(distance);  //Interval({distance})
            Interval& dist = cn.create_dom(distance);
            Interval& ttime = cn.create_dom(t);
            // Adding related observation constraints to the network
            IntervalVector& X_tube_ptr = cn.create_dom(IntervalVector(3));
            cn.add(ctc_distance, {cn.subvector(X_tube_ptr,0,1), UnknownBeacons[j],  dist});
            cn.add(ctc_evaluation, {ttime, X_tube_ptr, x_tube, v_tube});

            // Updated last iteration time
            prev_t_obs = t;
            i++ ;
        }
        double contraction_dt = cn.contract_during(iteration_dt,true);
        usleep(max(0.,iteration_dt-contraction_dt)*1e6); // pause for the animation

        // Display the current slice [x](t)
        figure.draw_box(x_tube(max(0.,ibex::previous_float(t))).subvector(0,1));
    }

    //figure.add_tube(&x_tube_contrated, "x_new", 0, 1);
    for(const auto& Mi : M)
        figure.add_beacon(Mi,0.1, "red[orange]");
    figure.show(0.5);

    // Show estimated boxes for Landmarks
    for(const auto& B : UnknownBeacons)
        figure.draw_box(B);
    figure.smooth_tube_drawing(true);
    figure.show();
    vibes::endDrawing();
}
