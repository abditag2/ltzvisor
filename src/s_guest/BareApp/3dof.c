#include "inc/3dof.h"
#include "inc/util.h"
#include <stdio.h>

// These are included in 3dof.h
//#include "geometry.h"
//#include "dynamics_3dof.h"


void restartedComputation() {
    // close and open the files to clear them

    return;

}

// called on states reached during the computation
bool intermediateState(HyperRectangle *r) {

    if (r->dims[0].max > 0.4 || r->dims[0].min < -0.4)
        return false;

    if (r->dims[1].max > 3.1415 / 4.0 || r->dims[1].min < -3.1415 / 4.0)
        return false;

    if (-r->dims[0].min + 0.33 * r->dims[1].max > 0.3 ||
        -r->dims[0].min - 0.33 * r->dims[1].max > 0.3)
        return false;

    if (-r->dims[0].max + 0.33 * r->dims[1].max > 0.3 ||
        -r->dims[0].max - 0.33 * r->dims[1].max > 0.3)
        return false;

    if (-r->dims[0].min + 0.33 * r->dims[1].min > 0.3 ||
        -r->dims[0].min - 0.33 * r->dims[1].min > 0.3)
        return false;

    if (-r->dims[0].max + 0.33 * r->dims[1].min > 0.3 ||
        -r->dims[0].max - 0.33 * r->dims[1].min > 0.3)
        return false;


    return true;

}

bool checkStabilityWithSC(HyperRectangle *r, double reachTimeSC) {
    double stepSize = 0.1;

    double points[64][6];
    int number_of_points = fill_in_the_critical_points_3dof(points, r);

    double rv;
    for (int i = 0; i < number_of_points; i++) {

        double time = 0;

        double p[6];

        for (int k = 0; k < 6; k++) {
            p[k] = points[i][k];
        }

        while (true) {
            if (shouldStopWithSafety(p, time, &rv))
                break;

            HyperRectangle rect;

            for (int d = 0; d < NUM_DIMS; ++d)
                rect.dims[d].min = rect.dims[d].max = p[d];

            // euler's method

            for (int d = 0; d < NUM_DIMS; ++d) {

                double der = get_derivative_bounds(&rect, 2 * d,
                                                   SIMPLE_CONTROLLER);

                p[d] += stepSize * der;
            }

            time += stepSize;
        }

        if (rv < 0) {
            return false;
        }

    }
    return true;

}

double
potential(double e, double p, double y, double de, double dp, double dy) {
    double state[6] = {e, p, y, de, dp, dy};

    double P_lmi[6][6] =
            {

                    {25.4636357727033,    -0.11459836805098,  0,                 4.28913430335527,     -0.0202143458739051,  0},
                    {-0.11459836805098,   2.93322110122599,   0 -
                                                              0.110071227498266, 0.26291287971907,     0},
                    {0,                   0,                  0,                 0,                    0,                    0},
                    {4.28913430335527,    -0.110071227498266, 0,                 5.76346166888949,     -0.00221851844466635, 0},
                    {-0.0202143458739051, 0.26291287971907,   0,                 -0.00221851844466635, 0.202523616289283,    0},
                    {0,                   0,                  0,                 0,                    0,                    0}
            };
    // X^t * P * x

    double xTransposeTimesP[6] = {0, 0, 0, 0, 0, 0};

    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col)
            xTransposeTimesP[row] += P_lmi[col][row] * state[col];
    }

    double rv = 0;

    for (int row = 0; row < 6; ++row)
        rv += xTransposeTimesP[row] * state[row];

    return rv;
}


bool finalState(HyperRectangle *rect) {
    double points[64][6];

    int number_of_points = fill_in_the_critical_points_3dof(points, rect);

    double maxPotential = potential(points[0][0], points[0][1], points[0][2],
                                    points[0][3], points[0][4], points[0][5]);

    for (int i = 1; i < number_of_points; ++i) {
        double p = potential(points[i][0], points[i][1], points[i][2],
                             points[i][3], points[i][4], points[i][5]);

        if (p > maxPotential)
            maxPotential = p;
    }

//	hyperrectangle_to_file(f_final, rect,2);
//    printf("--->  potential of final state = %f\n", maxPotential);

    return maxPotential < 1;
}

bool shouldStopWithSafety(double state[NUM_DIMS], double simTime, void *p) {

    bool rv = false;
    double pot = potential(state[0], state[1], state[2], state[3], state[4],
                           state[5]);
    double maxTime = 10.0;

    if (pot < 1) {
        rv = true;
        double *stopTime = (double *) p;
        *stopTime = simTime;
    }

    if (simTime >= maxTime) {
        rv = true;

        double *stopTime = (double *) p;
        *stopTime = -1;
    }

    if (-state[0] - (1 / 3) * state[1] >= 0.3 ||
        -state[0] + (1 / 3) * state[1] >= 0.3 || state[0] < -0.4) {
        double *stopTime = (double *) p;
        *stopTime = -1;
        rv = true;
    }

    return rv;
}

bool shouldStop(double state[NUM_DIMS], double simTime, void *p) {
    bool rv = false;
    double pot = potential(state[0], state[1], state[2], state[3], state[4],
                           state[5]);
    double maxTime = 10.0;

    if (pot < 1) {
        rv = true;

        double *stopTime = (double *) p;
        *stopTime = simTime;
    }

    if (simTime >= maxTime) {
        rv = true;

        double *stopTime = (double *) p;
        *stopTime = -1;
    }

    return rv;
}

bool runReachability(double *start, double reachTimeCC, double reachTimeSC,
                     double startMs) {
    LiftingSettings set;

    for (int d = 0; d < NUM_DIMS; ++d) {
        set.init.dims[d].min = start[d];
        set.init.dims[d].max = start[d];
    }

    set.reachTimeCC = reachTimeCC;
    set.reachTimeSC = reachTimeSC;

    set.initialStepSizeSC = set.reachTimeSC / 20;
//    set.initialStepSizeCC = set.reachTimeCC / 20;
    set.initialStepSizeCC = 0.1;

    set.maxRectWidthBeforeError = 100;

    set.reachedAtFinalTime = finalState;
    set.reachedAtIntermediateTime = intermediateState;

    set.checkStabilizabilityAfterCCperiod = checkStabilityWithSC;

//    HyperRectangle trackedRectAfterCC;
//    HyperRectangle trackedRectAfterSC;

    bool safe = true;

    safe = face_lifting_iterative_improvement(startMs, &set);

    return safe;
}


double findMaxRestartTime(double state[NUM_DIMS]) {

    double simTime;

    bool safe = false;
    double reachTimeCC = 0;
    double reachTimeSC = TIME_TO_RUN_SC;

    // check if it is possible at all to restart the system
    safe = isSafe(state, reachTimeCC, reachTimeSC, &simTime);

    if (!safe) {
        return -1;
    }

    //then find the minimum restart time
    double prevSafeReachTimeCC = 0;
    reachTimeCC = 0.8;

    for (int k = 0; k < 5; k++) {
        safe = isSafe(state, reachTimeCC, reachTimeSC, &simTime);

        if (safe) {
            prevSafeReachTimeCC = reachTimeCC;
            reachTimeCC = 2 * reachTimeCC;
        } else {
            reachTimeCC = (reachTimeCC + prevSafeReachTimeCC) / 2;
        }
    }

    return prevSafeReachTimeCC;
}

bool isSafe(double state[NUM_DIMS], double reachTimeCC, double reachTimeSC,
            double *simTime) {
    bool rv = false;

    int startMs = milliseconds();

    rv = runReachability(state, reachTimeCC, reachTimeSC, startMs);
    return rv;
}

