package org.firstinspires.ftc.teamcode.utility.RK4;

public class TrajectorySimulator {

    private static final double DEFAULT_DT = 0.001;
    private static final double MAX_FLIGHT_TIME = 5.0;

    private double dt;

    public TrajectorySimulator() {
        this.dt = DEFAULT_DT;
    }

    public TrajectorySimulator(double dt) {
        this.dt = dt;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public TrajectoryResult simulate(double turretPhi, double theta,
                                      RobotState robotState,
                                      ProjectileParameters params) {
        return simulate(turretPhi, theta, robotState.x, robotState.y, 0,
                       robotState.vx, robotState.vy, params);
    }

    public TrajectoryResult simulate(double turretPhi, double theta,
                                      double startX, double startY, double startZ,
                                      double robotVx, double robotVy,
                                      ProjectileParameters params) {

        ProjectileState state = computeInitialState(turretPhi, theta, startX, startY, 0,
                                                      robotVx, robotVy, params);

        double targetZ = params.deltaH;
        double prevZ = state.z;

        int maxSteps = (int) (MAX_FLIGHT_TIME / dt);

        for (int i = 0; i < maxSteps; i++) {
            state = rk4Step(state, params);
            state.time += dt;

            if (state.z >= targetZ && prevZ < targetZ) {
                double tFraction = (targetZ - prevZ) / (state.z - prevZ);
                double interpolatedX = state.x - (state.x - prevZ) * (1 - tFraction);
                double interpolatedY = state.y - (state.y - prevZ) * (1 - tFraction);
                double interpolatedTime = state.time - dt + dt * (1 - tFraction);

                return new TrajectoryResult(
                    interpolatedX,
                    interpolatedY,
                    targetZ,
                    interpolatedTime,
                    true,
                    theta,
                    turretPhi
                );
            }

            prevZ = state.z;

            if (state.z < targetZ - 0.5) {
                break;
            }
        }

        return new TrajectoryResult(
            state.x,
            state.y,
            state.z,
            state.time,
            false,
            theta,
            turretPhi
        );
    }

    private ProjectileState computeInitialState(double turretPhi, double theta,
                                                 double startX, double startY, double startZ,
                                                 double robotVx, double robotVy,
                                                 ProjectileParameters params) {
        double vx0 = params.v0 * Math.cos(theta) * Math.cos(turretPhi) + robotVx;
        double vy0 = params.v0 * Math.cos(theta) * Math.sin(turretPhi) + robotVy;
        double vz0 = params.v0 * Math.sin(theta);

        return new ProjectileState(startX, startY, startZ, vx0, vy0, vz0, 0);
    }

    private ProjectileState rk4Step(ProjectileState state, ProjectileParameters params) {
        double[] k1 = ProjectileDynamics.computeDerivatives(state, params);

        ProjectileState state2 = addDerivative(state, k1, dt * 0.5, params);
        double[] k2 = ProjectileDynamics.computeDerivatives(state2, params);

        ProjectileState state3 = addDerivative(state, k2, dt * 0.5, params);
        double[] k3 = ProjectileDynamics.computeDerivatives(state3, params);

        ProjectileState state4 = addDerivative(state, k3, dt, params);
        double[] k4 = ProjectileDynamics.computeDerivatives(state4, params);

        return new ProjectileState(
            state.x + (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) * dt / 6,
            state.y + (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) * dt / 6,
            state.z + (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) * dt / 6,
            state.vx + (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) * dt / 6,
            state.vy + (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]) * dt / 6,
            state.vz + (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]) * dt / 6,
            state.time + dt
        );
    }

    private ProjectileState addDerivative(ProjectileState state, double[] deriv, double factor,
                                          ProjectileParameters params) {
        return new ProjectileState(
            state.x + deriv[0] * factor,
            state.y + deriv[1] * factor,
            state.z + deriv[2] * factor,
            state.vx + deriv[3] * factor,
            state.vy + deriv[4] * factor,
            state.vz + deriv[5] * factor,
            state.time
        );
    }

    public static class TrajectoryResult {
        public final double landingX;
        public final double landingY;
        public final double landingZ;
        public final double flightTime;
        public final boolean reachedTargetHeight;
        public final double theta;
        public final double turretPhi;

        public TrajectoryResult(double landingX, double landingY, double landingZ,
                                 double flightTime, boolean reachedTargetHeight,
                                 double theta, double turretPhi) {
            this.landingX = landingX;
            this.landingY = landingY;
            this.landingZ = landingZ;
            this.flightTime = flightTime;
            this.reachedTargetHeight = reachedTargetHeight;
            this.theta = theta;
            this.turretPhi = turretPhi;
        }

        public double getHorizontalDistance() {
            return Math.sqrt(landingX * landingX + landingY * landingY);
        }

        public double getHorizontalError(double targetX, double targetY) {
            return Math.sqrt(
                Math.pow(landingX - targetX, 2) +
                Math.pow(landingY - targetY, 2)
            );
        }
    }
}