#include "inc/dynamics.h"


#ifdef DYNAMICS_PENDULUM

// a saturated controlled inverted pendulum
// X' = AX + Bu, with u = KX, u between -4.95 and 4.95
// where X = [pos_error velocity angle angular_vel]^T

// from lmi solving:
static const double K[2][6] =
        {
                {-5.6271,   -1.7337,   -0.0000,   -2.7299,   -0.4601,   -0.0000},
                {-5.7380,    1.8612,    0.0000,   -2.7501,    0.4812,    0.0000}
        };


int numDerivativeBoundsCalls = 0;


int fill_in_the_critical_points_3dof(double (*points)[6], HyperRectangle *rect) {
    int i0, i1, i2, i3, i4, i5;

    int number_of_points = 0;

    for (i0 = 0; i0 < 2; i0++) {
        for (i1 = 0; i1 < 2; i1++) {
            for (i2 = 0; i2 < 2; i2++) {
                for (i3 = 0; i3 < 2; i3++) {
                    for (i4 = 0; i4 < 2; i4++) {
                        for (i5 = 0; i5 < 2; i5++) {
                            if (i0 == 0)
                                points[number_of_points][0] = rect->dims[0].min;
                            else
                                points[number_of_points][0] = rect->dims[0].max;

                            if (i1 == 0)
                                points[number_of_points][1] = rect->dims[1].min;
                            else
                                points[number_of_points][1] = rect->dims[1].max;

                            if (i2 == 0)
                                points[number_of_points][2] = rect->dims[2].min;
                            else
                                points[number_of_points][2] = rect->dims[2].max;

                            if (i3 == 0)
                                points[number_of_points][3] = rect->dims[3].min;
                            else
                                points[number_of_points][3] = rect->dims[3].max;

                            if (i4 == 0)
                                points[number_of_points][4] = rect->dims[4].min;
                            else
                                points[number_of_points][4] = rect->dims[4].max;


                            if (i5 == 0)
                                points[number_of_points][5] = rect->dims[5].min;
                            else
                                points[number_of_points][5] = rect->dims[5].max;

                            number_of_points++;
                        }
                    }
                }
            }
        }
    }
    return number_of_points;
}


double eval_dim_with_controller_with_commad(int dim, double state[], double command[]) {

//    double p1 = -1.0000;
//    double p2 = -2.4000;
//    double p3 = -0.0943;
//    double p4 = 0.1200;
//    double p5 = 0.1200;
//    double p6 = -2.5000;
//    double p7 = -0.0200;
    double p8 = 0.200;
    double p9 = 2.1000;
//    double p10 = 10.0000;

//    double e = state[0];
//    double p = state[1];
//    double y = state[2];
    double de = state[3];
    double dp = state[4];
    double dy = state[5];

    double rv = 0;

    if (dim == 0) {
        rv = de;
    } else if (dim == 1) {
        rv = dp;
    } else if (dim == 2) {
        rv = dy;
    } else if (dim == 3) {
        rv = p8 * command[0] + p8 * command[1];
    } else if (dim == 4) {
        rv = p9 * command[0] - p9 * command[1];
    } else if (dim == 5) {
        rv = 0;
    }

    return rv;
}


int control_commands(double (*commands)[2], double point[], int controller_type) {

    int num_commands = 0;

    if (controller_type == COMPLEX_CONTROLLER) {

        commands[0][0] = U_MAX;
        commands[0][1] = U_MAX;

        commands[1][0] = U_MAX;
        commands[1][1] = U_MIN;

        commands[2][0] = U_MIN;
        commands[2][1] = U_MIN;

        commands[3][0] = U_MIN;
        commands[3][1] = U_MAX;

        num_commands = 4;

    } else if (controller_type == SIMPLE_CONTROLLER) {

//        printf("state: %f %f %f %f %f %f\n", point[0],point[1],point[2],point[3],point[4],point[5]);
//        printf("K1 %f %f %f %f %f %f\n", K[0][0], K[0][1], K[0][2], K[0][3], K[0][4], K[0][5]);
//        printf("K2 %f %f %f %f %f %f\n", K[1][0], K[1][1], K[1][2], K[1][3], K[1][4], K[1][5]);

        double com1 = K[0][0] * point[0] + K[0][1] * point[1] + K[0][2] * point[2] + K[0][3] * point[3] + K[0][4] * point[4] +
                      K[0][5] * point[5];

        commands[0][0] = com1;

        double com2 =K[1][0] * point[0] + K[1][1] * point[1] + K[1][2] * point[2] + K[1][3] * point[3] + K[1][4] * point[4] +
                     K[1][5] * point[5];


        commands[0][1] = com2;

//        printf("commands0 %f, commands1: %f\n", com1, com2);

        if (com1 > U_MAX)
            commands[0][0] = U_MAX;
        else if (com1 < U_MIN)
            commands[0][0] = U_MIN;

        if (com2 > U_MAX)
            commands[0][1] = U_MAX;
        else if (com2 < U_MIN)
            commands[0][1] = U_MIN;

        num_commands = 1;
    }

    return num_commands;

}


double get_derivative_bounds(HyperRectangle *rect, int faceIndex, int controller_type) {

    ++numDerivativeBoundsCalls;

    int dim = faceIndex / 2;
    bool isMin = (faceIndex % 2) == 0;

    double rv = 0;

    double points[256][6];
    int number_of_points = fill_in_the_critical_points_3dof(points, rect);

    double commands[4][2];

    int num_commands = control_commands(commands, points[0], controller_type);
    rv = eval_dim_with_controller_with_commad(dim, points[0], commands[0]);

    for (int i = 0; i < number_of_points; ++i) {

        num_commands = control_commands(commands, points[i], controller_type);

//        if (controller_type == SIMPLE_CONTROLLER){
//            printf("\n");
//            printf("%f %f %f %f", commands[0][0],commands[0][1],commands[1][0],commands[1][1]);
//        }

        int k = 0;
        for (k = 0; k < num_commands; k++) {

            double val = eval_dim_with_controller_with_commad(dim, points[i], commands[k]);

            if (isMin && val < rv) {
                rv = val;
            } else if (!isMin && val > rv)
                rv = val;
        }
    }


    return rv;
}


#endif
