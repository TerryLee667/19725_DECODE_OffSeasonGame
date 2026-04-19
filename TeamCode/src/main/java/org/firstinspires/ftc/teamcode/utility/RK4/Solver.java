package org.firstinspires.ftc.teamcode.utility.RK4;

public class Solver {

    private TrajectorySimulator simulator;
    private ProjectileParameters params;

    private int maxOuterIterations;
    private double phiTolerance;
    private int maxBinarySearchIterations;
    private double binarySearchTolerance;

    public Solver() {
        this.simulator = new TrajectorySimulator();
        this.params = new ProjectileParameters();
        this.maxOuterIterations = 5;
        this.phiTolerance = Math.toRadians(0.5);
        this.maxBinarySearchIterations = 15;
        this.binarySearchTolerance = 0.001;
    }

    public Solver(ProjectileParameters params) {
        this.simulator = new TrajectorySimulator();
        this.params = params;
        this.maxOuterIterations = 5;
        this.phiTolerance = Math.toRadians(0.5);
        this.maxBinarySearchIterations = 15;
        this.binarySearchTolerance = 0.001;
    }

    public void setParameters(ProjectileParameters params) {
        this.params = params;
    }

    public void setMaxOuterIterations(int max) {
        this.maxOuterIterations = max;
    }

    public void setPhiTolerance(double toleranceRadians) {
        this.phiTolerance = toleranceRadians;
    }

    public void setMaxBinarySearchIterations(int max) {
        this.maxBinarySearchIterations = max;
    }

    public void setBinarySearchTolerance(double tolerance) {
        this.binarySearchTolerance = tolerance;
    }

    public SolverResult solve(double targetX, double targetY,
                               RobotState robotState,
                               TargetPredictor.TargetState targetState) {
        double dx = targetX - robotState.x;
        double dy = targetY - robotState.y;

        double initialPhi = Math.atan2(dy, dx);
        return solveWithInitialPhi(initialPhi, targetX, targetY, robotState, targetState);
    }

    public SolverResult solve(double relativeX, double relativeY, double robotVx, double robotVy) {
        RobotState robotState = new RobotState(0, 0, robotVx, robotVy);
        TargetPredictor.TargetState targetState = new TargetPredictor.TargetState(relativeX, relativeY, 0, 0);
        
        double initialPhi = Math.atan2(relativeY, relativeX);
        return solveWithInitialPhi(initialPhi, relativeX, relativeY, robotState, targetState);
    }

    public SolverResult solveWithInitialPhi(double initialPhi,
                                             double targetX, double targetY,
                                             RobotState robotState,
                                             TargetPredictor.TargetState targetState) {
        double turretPhi = initialPhi;
        double bestTheta = 0;
        double bestFlightTime = 0;
        double predictedTargetX = targetX;
        double predictedTargetY = targetY;

        for (int i = 0; i < maxOuterIterations; i++) {
            double phiBefore = turretPhi;

            BinarySearchResult binaryResult = binarySearchElevation(turretPhi, predictedTargetX,
                                                                     predictedTargetY, robotState);

            if (!binaryResult.found) {
                return new SolverResult(turretPhi, bestTheta, bestFlightTime, false,
                                        "Elevation search failed to find solution");
            }

            bestTheta = binaryResult.theta;
            bestFlightTime = binaryResult.flightTime;

            predictedTargetX = targetX + targetState.vx * bestFlightTime;
            predictedTargetY = targetY + targetState.vy * bestFlightTime;

            double dx = predictedTargetX - robotState.x;
            double dy = predictedTargetY - robotState.y;
            double groundAngle = Math.atan2(dy, dx);

            double vHorizontal = params.v0 * Math.cos(bestTheta);
            double correction = calculatePhiCorrection(groundAngle, turretPhi, robotState, vHorizontal);

            turretPhi = groundAngle - correction;

            if (Math.abs(turretPhi - phiBefore) < phiTolerance) {
                break;
            }
        }

        return new SolverResult(turretPhi, bestTheta, bestFlightTime, true, "Success");
    }

    private double calculatePhiCorrection(double groundAngle, double turretPhi,
                                         RobotState robotState, double vHorizontal) {
        double vRx = robotState.vx;
        double vRy = robotState.vy;

        double correction = (vRy * Math.cos(groundAngle) - vRx * Math.sin(groundAngle)) / vHorizontal;

        return correction;
    }

    private BinarySearchResult binarySearchElevation(double turretPhi,
                                                     double targetX, double targetY,
                                                     RobotState robotState) {
        double low = 0;
        double high = params.thetaMax;
        double bestTheta = 0;
        double bestError = Double.MAX_VALUE;
        double bestFlightTime = 0;
        double bestLandingX = 0;
        double bestLandingY = 0;

        for (int i = 0; i < maxBinarySearchIterations; i++) {
            double mid = (low + high) / 2;

            TrajectorySimulator.TrajectoryResult result = simulator.simulate(
                turretPhi, mid,
                robotState.x, robotState.y, 0,
                robotState.vx, robotState.vy,
                params
            );

            if (!result.reachedTargetHeight) {
                high = mid;
                continue;
            }

            double dx = result.landingX - targetX;
            double dy = result.landingY - targetY;
            double error = Math.sqrt(dx * dx + dy * dy);

            if (error < bestError) {
                bestError = error;
                bestTheta = mid;
                bestFlightTime = result.flightTime;
                bestLandingX = result.landingX;
                bestLandingY = result.landingY;
            }

            double predictedDistance = Math.sqrt(
                Math.pow(targetX - robotState.x, 2) +
                Math.pow(targetY - robotState.y, 2)
            );
            double landingDistance = result.getHorizontalDistance();

            if (Math.abs(error) < binarySearchTolerance) {
                return new BinarySearchResult(bestTheta, bestFlightTime, true, bestError);
            }

            if (landingDistance < predictedDistance) {
                low = mid;
            } else {
                high = mid;
            }
        }

        return new BinarySearchResult(bestTheta, bestFlightTime, bestError < 0.5, bestError);
    }

    public static class BinarySearchResult {
        public final double theta;
        public final double flightTime;
        public final boolean found;
        public final double finalError;

        public BinarySearchResult(double theta, double flightTime, boolean found, double finalError) {
            this.theta = theta;
            this.flightTime = flightTime;
            this.found = found;
            this.finalError = finalError;
        }
    }

    public static class SolverResult {
        public final double turretPhi;
        public final double theta;
        public final double flightTime;
        public final boolean success;
        public final String message;

        public SolverResult(double turretPhi, double theta, double flightTime,
                           boolean success, String message) {
            this.turretPhi = turretPhi;
            this.theta = theta;
            this.flightTime = flightTime;
            this.success = success;
            this.message = message;
        }

        public double getTurretPhiDegrees() {
            return Math.toDegrees(turretPhi);
        }

        public double getThetaDegrees() {
            return Math.toDegrees(theta);
        }

        public boolean isOutOfRange() {
            return theta > Math.PI / 4;
        }
    }
}