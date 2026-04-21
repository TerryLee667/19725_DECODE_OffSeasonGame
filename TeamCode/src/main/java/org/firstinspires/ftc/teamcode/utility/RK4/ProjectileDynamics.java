package org.firstinspires.ftc.teamcode.utility.RK4;

public class ProjectileDynamics {

    public static double[] computeDerivatives(ProjectileState state, ProjectileParameters params) {
        double speed = state.getSpeed();

        if (speed < 1e-10) {
            return new double[]{
                state.vx,
                state.vy,
                state.vz,
                0,
                0,
                -params.g
            };
        }

        double dragFactor = -params.k / params.m * Math.pow(speed, params.n - 1);

        double ax = dragFactor * state.vx;
        double ay = dragFactor * state.vy;
        double az = dragFactor * state.vz - params.g;

        return new double[]{
            state.vx,
            state.vy,
            state.vz,
            ax,
            ay,
            az
        };
    }

    public static ProjectileState derivativeToState(double[] deriv, double time) {
        return new ProjectileState(
            deriv[0],
            deriv[1],
            deriv[2],
            deriv[3],
            deriv[4],
            deriv[5],
            time
        );
    }
}