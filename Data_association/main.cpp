#include <tubex.h>

#include <unistd.h>

using namespace std;
using namespace ibex;
using namespace tubex;

class MyCtc : public ibex::Ctc
{
  public:

    MyCtc(const std::vector<ibex::IntervalVector>& M_)
      : ibex::Ctc(2), // the contractor acts on 2d boxes
        M(M_)         // attribute needed later on for the contraction
    {

    }

    void contract(ibex::IntervalVector& a)
    {
      /* ------ Contraction formula : D.2 -------*/

        IntervalVector result;
        int i = 1;
        for(auto& Mi : M)
        {
            if (i>1) {
                result |=  (a & Mi);}
            else {
                result =  (a & Mi);}
            i++;
        }
        a = result;
    }

  protected:

    const std::vector<ibex::IntervalVector> M;
};

// Wait function to pause between plotting and smoothing tube of x

inline void wait_on_enter()
{
    std::string dummy;
    std::cout << "Enter to continue smoothing tubes ..." << std::endl;
    std::getline(std::cin, dummy);
}



/* ------ -------- ------------- -------
 * ----------    MAIN    ---------------
 * ----------------------------------*/

int main()
  {

    double dt = 0.05;
    Interval tdomain(0,6);
    TrajectoryVector actual_x(tdomain,TFunction("(10*cos(t)+t ; 5*sin(2*t)+t;\
                                                atan2((10*cos(2*t)+1),(-10*sin(t)+1));\
                                                sqrt((10*cos(2*t)+1)^2+(-10*sin(t)+1)^2))"), dt);

    vibes::beginDrawing();
    VIBesFigMap figure("Estimation with Data Association");
    figure.set_properties(50, 50, 600, 400);
    figure.add_trajectory(&actual_x, "x*", 0, 1, 2);

    figure.axis_limits(IntervalVector({{-8,11},{-4.,8.}}));
    figure.show(1.0);

    //vibes::saveImage();

    // Creating random map of landmarks
    int nb_landmarks = 150;
    IntervalVector map_area(actual_x.codomain().subvector(0,1));
    //cout << "map_area.codomain" << map_area << endl;

    map_area.inflate(2);
    vector<IntervalVector> v_map =
      DataLoader::generate_landmarks_boxes(map_area, nb_landmarks);

    // Generating observations obs=(t,range,bearing) of these landmarks
    int max_nb_obs = 20;
    Interval visi_range(0,4); // [0m,75m]
    Interval visi_angle(-M_PI/4,M_PI/4); // frontal sonar

    vector<IntervalVector> v_obs =
      DataLoader::generate_observations(actual_x, v_map, max_nb_obs,
                                        true, visi_range, visi_angle);

    double epsilon_range = 0.1;
    double epsilon_bearing = 0.04;

    for(auto& obs : v_obs)
    {
        // Adding uncertainties on the measurements
        obs[1].inflate(epsilon_range); // range
        obs[2].inflate(epsilon_bearing); // bearing
    }


    // Display landmarks and observations
    for(const auto& landmark : v_map)
        figure.add_beacon(landmark,0.1, "red[orange]");
    figure.axis_limits(figure.view_box(), true, 0.1);
    figure.show();

    //vibes::saveImage();

    /*---------------------------------------------*/
    /*  State  estimation with data association    */
    /*---------------------------------------------*/

    // Contractors
    CtcFunction ctc_plus(Function("a", "b", "c", "a+b-c")); // a+b=c
    CtcFunction ctc_minus(Function("a", "b", "c", "a-b-c")); // a-b=c

    CtcFunction ctc_systemEquation(
      Function("x[4]","u[2]","v[4]","(x[3]*cos(x[2])-v[0]; x[3]*sin(x[2])-v[1]; u[0]-v[2]; u[1]-v[3])"));

    CtcEval ctc_evaluation;

    CtcDeriv ctc_derivation;

    MyCtc ctc_dataAssociation(v_map);



    // Intermediate variables
    int nb_obs = v_obs.size();
    IntervalVector angle_polar(nb_obs);
    IntervalMatrix distance_polar(nb_obs,2);

    // Association set (possible identities)
    vector<IntervalVector> UnknownBeacons(nb_obs, IntervalVector(2));
    // unknown association for each observation

    for(auto& UB : UnknownBeacons)
        cout << "the initialisation of  UnknownBeacon " << UB << endl;
    //-------------------------

    TubeVector x(tdomain, dt, 4);
    TubeVector u(tdomain, dt, 2);
    TubeVector v(tdomain, dt, 4);

    double epsilonx = 0.01;
    Tube psix(actual_x[2], dt);
    Tube thetax(actual_x[3], dt);

    psix.inflate(epsilonx);
    thetax.inflate(epsilonx);

    x[2] = psix;               // the heading is known
    x[3] = thetax;

    //-----------------------------------------------------------------------

    ContractorNetwork cn;

    cn.add(ctc_systemEquation, {x, u, v});
    cn.add(ctc_derivation, {x, v});

    for(int i = 0 ; i < nb_obs ; i++) // for each measurement
    {
        IntervalVector& p = cn.create_dom(IntervalVector(4));
        // Solver
        cn.add(ctc_plus, {v_obs[i][2], cn.subvector(p,2,2), angle_polar[i]});   // [dist]+ truth angle = theta
        cn.add(ctc_minus, {UnknownBeacons[i][0], cn.subvector(p,0,0), distance_polar[i][0]});   // UnknownBeacon_x - X  = d0
        cn.add(ctc_minus, {UnknownBeacons[i][1], cn.subvector(p,1,1), distance_polar[i][1]});   // UnknownBeacon_y - Y = d1
        cn.add(ctc::polar, {distance_polar[i][0],distance_polar[i][1], v_obs[i][1], angle_polar[i]});
        cn.add(ctc_dataAssociation, {UnknownBeacons[i]});
        cn.add(ctc_evaluation, {v_obs[i][0], p, x, v});
    }

    cn.contract(true);


    figure.add_tube(&x, "x", 0, 1);
    for(const auto& obs : v_obs)
        figure.draw_vehicle(obs[0].mid(), &actual_x,0.6);
    figure.show(1);

    //vibes::saveImage();

    //wait_on_enter();
    sleep(2);

    figure.smooth_tube_drawing(true);
    figure.add_observations(v_obs, &actual_x);
    figure.show();

    //vibes::saveImage();


    vibes::endDrawing();
}

