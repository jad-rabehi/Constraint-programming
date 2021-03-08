#include <tubex.h>


using namespace std;
using namespace tubex;


/* -----------------------------------------------
 * -----  Estimation based on asynchronous
 * -----           measurements
 * ----------------------------------------------- */


int main()
{

    double dt = 0.01;
    Interval tdomain(0,3);
    vector<double> t = {0.3, 1.5, 2};

    // Landmarks coordinates
    vector<Vector> b = {{8,3},{0,5},{-2,1}};

    IntervalVector y({{1.9},{3.6},{2.8}});
    int nb_landmarks = 3;


    // Landmarks' uncertainties
    for (auto& yi : y)
    {
        yi.inflate(0.1);
    }

    TrajectoryVector x_truth(tdomain,TFunction("(10*cos(t)+t ; 5*sin(2*t)+t;\
                                               atan2((10*cos(2*t)+1),(-10*sin(t)+1));\
                             sqrt((10*cos(2*t)+1)^2+(-10*sin(t)+1)^2))"), dt);


    vibes::beginDrawing();
    VIBesFigMap figure("Estimation based on Asynchronous Measurement");
    figure.set_properties(50, 50, 600, 400);
    figure.add_trajectory(&x_truth, "x*", 0, 1, 2);

    for(int i = 0 ; i < nb_landmarks ; i++)
    {
        figure.draw_ring(b[i][0], b[i][1], y[i], "gray");
        figure.add_beacon(b[i],0.2,"red[yellow]");
    }

    //fig.axis_limits(-2.5,2.5,-0.1,0.1, true);
    figure.axis_limits(IntervalVector({{-8,11},{-4.,8.}}));
    figure.show(1);

    TubeVector x(tdomain, dt, 4);
    TubeVector u(tdomain, dt, 2);
    TubeVector v(tdomain, dt, 4);

    double epsilon = 0.01;
    Tube psi(x_truth[2], dt);
    Tube theta(x_truth[3], dt);

    psi.inflate(epsilon);
    theta.inflate(epsilon);

    x[2] = psi;
    x[3] = theta;


    CtcFunction ctc_systemEquation(
                Function("x[4]","u[2]","v[4]","(x[3]*cos(x[2])-v[0]; x[3]*sin(x[2])-v[1]; u[0]-v[2]; u[1]-v[3])"));

    CtcEval ctc_evaluation;

    CtcFunction ctc_distance(Function("x[2]", "bk[2]", "dk",
                                      "sqrt((x[0]-bk[0])^2+(x[1]-bk[1])^2)-dk"));  // Contractor for distance

    // Create the network of contactors
    ContractorNetwork cn;

    // Add the system equation constraint
    cn.add(ctc_systemEquation, {x, u, v});



    IntervalVector& Xref0 = cn.create_dom(IntervalVector(4));        // Create domain (-inf, inf) for the 1st unkown beacon
    cn.add(ctc_distance, {cn.subvector(Xref0,0,1), b[0], y[0]});     // add distance contractor
    cn.add(ctc_evaluation, {t[0], Xref0, x, v});

    IntervalVector& Xref1 = cn.create_dom(IntervalVector(4));
    cn.add(ctc_distance, {cn.subvector(Xref1,0,1), b[1], y[1]});
    cn.add(ctc_evaluation, {t[1], Xref1, x, v});

    IntervalVector& Xref2 =cn.create_dom(IntervalVector(4));
    cn.add(ctc_distance, {cn.subvector(Xref2,0,1), b[2], y[2]});
    cn.add(ctc_evaluation, {t[2], Xref2, x, v});

    // Launch contraction
    cn.contract();


    figure.add_tube(&x, "x", 0, 1);
    figure.show();

    figure.add_beacon(Xref0,"yellow[yellow]");
    figure.add_beacon(Xref1,"yellow[yellow]");
    figure.add_beacon(Xref2,"yellow[yellow]");
    figure.show();

    vibes::endDrawing();

}
