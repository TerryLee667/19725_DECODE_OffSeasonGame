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

        // 计算阻力：kx * v（方向与v相反）
        double dragForceX = -params.kx * speed * state.vx / speed;
        double dragForceY = -params.kx * speed * state.vy / speed;
        double dragForceZ = -params.kx * speed * state.vz / speed;

        // 计算升力：ky * v（方向竖直向上）
        double liftForceZ = params.ky * speed;

        // 计算加速度
        double ax = dragForceX / params.m;
        double ay = dragForceY / params.m;
        double az = (dragForceZ + liftForceZ) / params.m - params.g;

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