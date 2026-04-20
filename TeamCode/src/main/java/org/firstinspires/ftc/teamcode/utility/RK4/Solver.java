package org.firstinspires.ftc.teamcode.utility.RK4;

import java.util.HashMap;
import java.util.Map;

public class Solver {

    private TrajectorySimulator simulator;
    private ProjectileParameters params;
    private Map<String, ProjectileParameters> parameterSets;
    private String currentParameterSet;

    private int maxOuterIterations;
    private double phiTolerance;
    private int maxBinarySearchIterations;
    private double binarySearchTolerance;
    
    // 模式相关的范围设置
    private double yawModeThetaMin;    // Yaw模式下仰角最小值
    private double yawModeThetaMax;    // Yaw模式下仰角最大值
    private double velModeThetaMin;    // Vel模式下仰角最小值
    private double velModeThetaMax;    // Vel模式下仰角最大值
    private double v0Min;              // 初速度最小值
    private double v0Max;              // 初速度最大值

    public Solver() {
        this.simulator = new TrajectorySimulator();
        this.params = new ProjectileParameters();
        this.parameterSets = new HashMap<>();
        this.parameterSets.put("default", params);
        this.currentParameterSet = "default";
        this.maxOuterIterations = 5;
        this.phiTolerance = Math.toRadians(0.5);
        this.maxBinarySearchIterations = 15;
        this.binarySearchTolerance = 0.001;
        
        // 初始化模式相关的范围设置
        this.yawModeThetaMin = 0;
        this.yawModeThetaMax = Math.PI / 4;  // Yaw模式：0-45度
        this.velModeThetaMin = 0;
        this.velModeThetaMax = Math.toRadians(55);  // Vel模式：0-55度
        this.v0Min = 2.0;
        this.v0Max = 15.0;
    }

    public Solver(ProjectileParameters params) {
        this.simulator = new TrajectorySimulator();
        this.params = params;
        this.parameterSets = new HashMap<>();
        this.parameterSets.put("default", params);
        this.currentParameterSet = "default";
        this.maxOuterIterations = 5;
        this.phiTolerance = Math.toRadians(0.5);
        this.maxBinarySearchIterations = 15;
        this.binarySearchTolerance = 0.001;
        
        // 初始化模式相关的范围设置
        this.yawModeThetaMin = 0;
        this.yawModeThetaMax = Math.PI / 4;  // Yaw模式：0-45度
        this.velModeThetaMax = Math.toRadians(55);  // Vel模式：0-55度
        this.velModeThetaMin = 0;
        this.v0Min = 2.0;
        this.v0Max = 15.0;
    }

    public void setParameters(ProjectileParameters params) {
        this.params = params;
        this.parameterSets.put(currentParameterSet, params);
    }
    
    public void addParameterSet(String name, ProjectileParameters params) {
        this.parameterSets.put(name, params);
    }
    
    public void switchParameterSet(String name) {
        if (parameterSets.containsKey(name)) {
            this.params = parameterSets.get(name);
            this.currentParameterSet = name;
        }
    }
    
    public ProjectileParameters getParameterSet(String name) {
        return parameterSets.get(name);
    }
    
    public ProjectileParameters getCurrentParameters() {
        return params;
    }
    
    public String getCurrentParameterSetName() {
        return currentParameterSet;
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
    
    // Yaw模式仰角范围设置
    public void setYawModeThetaRange(double min, double max) {
        this.yawModeThetaMin = min;
        this.yawModeThetaMax = max;
    }
    
    // Vel模式仰角范围设置
    public void setVelModeThetaRange(double min, double max) {
        this.velModeThetaMin = min;
        this.velModeThetaMax = max;
    }
    
    // 初速度范围设置
    public void setV0Range(double min, double max) {
        this.v0Min = min;
        this.v0Max = max;
    }
    
    // 获取Yaw模式仰角范围
    public double[] getYawModeThetaRange() {
        return new double[]{yawModeThetaMin, yawModeThetaMax};
    }
    
    // 获取Vel模式仰角范围
    public double[] getVelModeThetaRange() {
        return new double[]{velModeThetaMin, velModeThetaMax};
    }
    
    // 获取初速度范围
    public double[] getV0Range() {
        return new double[]{v0Min, v0Max};
    }

    public SolverResult solve(double targetX, double targetY,
                               RobotState robotState,
                               TargetPredictor.TargetState targetState,
                               double v0) {
        double dx = targetX - robotState.x;
        double dy = targetY - robotState.y;

        double initialPhi = Math.atan2(dy, dx);
        return solveWithInitialPhi(initialPhi, targetX, targetY, robotState, targetState, v0);
    }

    public SolverResult solve(double relativeX, double relativeY, double robotVx, double robotVy, double param, String mode) {
        RobotState robotState = new RobotState(0, 0, robotVx, robotVy);
        TargetPredictor.TargetState targetState = new TargetPredictor.TargetState(relativeX, relativeY, 0, 0);
        
        double initialPhi = Math.atan2(relativeY, relativeX);
        
        if (mode.equals("Yaw")) {
            // 传统模式：param 是初速度 v0
            return solveWithInitialPhi(initialPhi, relativeX, relativeY, robotState, targetState, param);
        } else if (mode.equals("Vel")) {
            // 新模式：param 是固定仰角 theta
            return solveWithFixedElevation(initialPhi, relativeX, relativeY, robotState, targetState, param);
        } else {
            throw new IllegalArgumentException("Mode must be 'Yaw' or 'Vel'");
        }
    }
    
    public SolverResult solve(double relativeX, double relativeY, double robotVx, double robotVy, double v0) {
        return solve(relativeX, relativeY, robotVx, robotVy, v0, "Yaw");
    }
    
    public SolverResult solve(double targetX, double targetY,
                               RobotState robotState,
                               TargetPredictor.TargetState targetState) {
        return solve(targetX, targetY, robotState, targetState, params.v0);
    }

    public SolverResult solve(double relativeX, double relativeY, double robotVx, double robotVy) {
        return solve(relativeX, relativeY, robotVx, robotVy, params.v0);
    }

    public SolverResult solveWithInitialPhi(double initialPhi,
                                             double targetX, double targetY,
                                             RobotState robotState,
                                             TargetPredictor.TargetState targetState,
                                             double v0) {
        double turretPhi = initialPhi;
        double bestTheta = 0;
        double bestFlightTime = 0;
        double predictedTargetX = targetX;
        double predictedTargetY = targetY;

        for (int i = 0; i < maxOuterIterations; i++) {
            double phiBefore = turretPhi;

            BinarySearchResult binaryResult = binarySearchElevation(turretPhi, predictedTargetX,
                                                                     predictedTargetY, robotState, v0);

            if (!binaryResult.found) {
                return new SolverResult(turretPhi, bestTheta, v0, bestFlightTime, false,
                                        "Elevation search failed to find solution");
            }

            bestTheta = binaryResult.theta;
            bestFlightTime = binaryResult.flightTime;

            predictedTargetX = targetX + targetState.vx * bestFlightTime;
            predictedTargetY = targetY + targetState.vy * bestFlightTime;

            double dx = predictedTargetX - robotState.x;
            double dy = predictedTargetY - robotState.y;
            double groundAngle = Math.atan2(dy, dx);

            double vHorizontal = v0 * Math.cos(bestTheta);
            double correction = calculatePhiCorrection(groundAngle, turretPhi, robotState, vHorizontal);

            turretPhi = groundAngle - correction;

            if (Math.abs(turretPhi - phiBefore) < phiTolerance) {
                break;
            }
        }

        return new SolverResult(turretPhi, bestTheta, v0, bestFlightTime, true, "Success");
    }
    
    public SolverResult solveWithInitialPhi(double initialPhi,
                                             double targetX, double targetY,
                                             RobotState robotState,
                                             TargetPredictor.TargetState targetState) {
        return solveWithInitialPhi(initialPhi, targetX, targetY, robotState, targetState, params.v0);
    }
    
    public SolverResult solveWithFixedElevation(double initialPhi,
                                               double targetX, double targetY,
                                               RobotState robotState,
                                               TargetPredictor.TargetState targetState,
                                               double fixedTheta) {
        // 检查仰角范围：Vel模式下0-55度
        if (fixedTheta < velModeThetaMin || fixedTheta > velModeThetaMax) {
            return new SolverResult(initialPhi, fixedTheta, 0, 0, false,
                                    "Elevation angle must be between " + Math.toDegrees(velModeThetaMin) + 
                                    " and " + Math.toDegrees(velModeThetaMax) + " degrees");
        }
        
        double turretPhi = initialPhi;
        double bestV0 = 0;
        double bestFlightTime = 0;
        double predictedTargetX = targetX;
        double predictedTargetY = targetY;

        for (int i = 0; i < maxOuterIterations; i++) {
            double phiBefore = turretPhi;

            BinarySearchVelocityResult velocityResult = binarySearchVelocity(turretPhi, predictedTargetX,
                                                                             predictedTargetY, robotState, fixedTheta);

            if (!velocityResult.found) {
                return new SolverResult(turretPhi, fixedTheta, bestV0, bestFlightTime, false,
                                        "Velocity search failed to find solution");
            }

            bestV0 = velocityResult.v0;
            bestFlightTime = velocityResult.flightTime;

            predictedTargetX = targetX + targetState.vx * bestFlightTime;
            predictedTargetY = targetY + targetState.vy * bestFlightTime;

            double dx = predictedTargetX - robotState.x;
            double dy = predictedTargetY - robotState.y;
            double groundAngle = Math.atan2(dy, dx);

            double vHorizontal = bestV0 * Math.cos(fixedTheta);
            double correction = calculatePhiCorrection(groundAngle, turretPhi, robotState, vHorizontal);

            turretPhi = groundAngle - correction;

            if (Math.abs(turretPhi - phiBefore) < phiTolerance) {
                break;
            }
        }

        return new SolverResult(turretPhi, fixedTheta, bestV0, bestFlightTime, true, "Success");
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
                                                     RobotState robotState,
                                                     double v0) {
        double low = yawModeThetaMin;
        double high = yawModeThetaMax;
        double bestTheta = 0;
        double bestError = Double.MAX_VALUE;
        double bestFlightTime = 0;
        double bestLandingX = 0;
        double bestLandingY = 0;

        // 创建临时参数对象，使用传入的 v0
        ProjectileParameters tempParams = params.copy();
        tempParams.v0 = v0;

        for (int i = 0; i < maxBinarySearchIterations; i++) {
            double mid = (low + high) / 2;

            TrajectorySimulator.TrajectoryResult result = simulator.simulate(
                turretPhi, mid,
                robotState.x, robotState.y, 0,
                robotState.vx, robotState.vy,
                tempParams
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
    
    private BinarySearchResult binarySearchElevation(double turretPhi,
                                                     double targetX, double targetY,
                                                     RobotState robotState) {
        return binarySearchElevation(turretPhi, targetX, targetY, robotState, params.v0);
    }
    
    private BinarySearchVelocityResult binarySearchVelocity(double turretPhi,
                                                           double targetX, double targetY,
                                                           RobotState robotState,
                                                           double fixedTheta) {
        double vMin = v0Min;   // 最小初速度 m/s
        double vMax = v0Max;  // 最大初速度 m/s
        double bestV0 = 0;
        double bestError = Double.MAX_VALUE;
        double bestFlightTime = 0;

        for (int i = 0; i < maxBinarySearchIterations; i++) {
            double vMid = (vMin + vMax) / 2;

            ProjectileParameters tempParams = params.copy();
            tempParams.v0 = vMid;

            TrajectorySimulator.TrajectoryResult result = simulator.simulate(
                turretPhi, fixedTheta,
                robotState.x, robotState.y, 0,
                robotState.vx, robotState.vy,
                tempParams
            );

            if (!result.reachedTargetHeight) {
                // 如果未达到目标高度，可能需要更大的初速度
                vMin = vMid;
                continue;
            }

            double dx = result.landingX - targetX;
            double dy = result.landingY - targetY;
            double error = Math.sqrt(dx * dx + dy * dy);

            if (error < bestError) {
                bestError = error;
                bestV0 = vMid;
                bestFlightTime = result.flightTime;
            }

            double predictedDistance = Math.sqrt(
                Math.pow(targetX - robotState.x, 2) +
                Math.pow(targetY - robotState.y, 2)
            );
            double landingDistance = result.getHorizontalDistance();

            if (Math.abs(error) < binarySearchTolerance) {
                return new BinarySearchVelocityResult(bestV0, bestFlightTime, true, bestError);
            }

            if (landingDistance < predictedDistance) {
                vMin = vMid;
            } else {
                vMax = vMid;
            }
        }

        return new BinarySearchVelocityResult(bestV0, bestFlightTime, bestError < 0.5, bestError);
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
    
    public static class BinarySearchVelocityResult {
        public final double v0;
        public final double flightTime;
        public final boolean found;
        public final double finalError;

        public BinarySearchVelocityResult(double v0, double flightTime, boolean found, double finalError) {
            this.v0 = v0;
            this.flightTime = flightTime;
            this.found = found;
            this.finalError = finalError;
        }
    }

    public static class SolverResult {
        public final double turretPhi;
        public final double theta;
        public final double v0;
        public final double flightTime;
        public final boolean success;
        public final String message;

        public SolverResult(double turretPhi, double theta, double v0, double flightTime,
                           boolean success, String message) {
            this.turretPhi = turretPhi;
            this.theta = theta;
            this.v0 = v0;
            this.flightTime = flightTime;
            this.success = success;
            this.message = message;
        }

        public SolverResult(double turretPhi, double theta, double flightTime,
                           boolean success, String message) {
            this(turretPhi, theta, 0, flightTime, success, message);
        }

        public double getTurretPhiDegrees() {
            return Math.toDegrees(turretPhi);
        }

        public double getThetaDegrees() {
            return Math.toDegrees(theta);
        }

        public double getV0() {
            return v0;
        }

        public boolean isOutOfRange() {
            return theta > Math.PI / 4;
        }
    }
}