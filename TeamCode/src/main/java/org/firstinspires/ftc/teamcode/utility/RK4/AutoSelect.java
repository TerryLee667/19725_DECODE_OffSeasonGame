package org.firstinspires.ftc.teamcode.utility.RK4;

import java.util.ArrayList;
import java.util.List;

public class AutoSelect {

    private Solver solver;
    private ProjectileParameters params;
    private List<Double> optimalV0List;
    private List<Double> optimalThetaList;
    private double v0Min;
    private double v0Max;
    private double thetaMin;
    private double thetaMax;

    public AutoSelect() {
        this.solver = new Solver();
        this.params = new ProjectileParameters();
        this.optimalV0List = new ArrayList<>();
        this.optimalThetaList = new ArrayList<>();
        this.v0Min = 2.0;
        this.v0Max = 15.0;
        this.thetaMin = 0;
        this.thetaMax = Math.PI / 4;
        
        // 初始化最优初速度列表和最优仰角列表
        initializeOptimalLists();
    }

    public AutoSelect(ProjectileParameters params) {
        this.solver = new Solver(params);
        this.params = params;
        this.optimalV0List = new ArrayList<>();
        this.optimalThetaList = new ArrayList<>();
        this.v0Min = 2.0;
        this.v0Max = 15.0;
        this.thetaMin = 0;
        this.thetaMax = Math.PI / 4;
        
        // 初始化最优初速度列表和最优仰角列表
        initializeOptimalLists();
    }

    private void initializeOptimalLists() {
        // 最优初速度列表：根据实际测试结果调整
        optimalV0List.add(7.5); // 首选初速度
        optimalV0List.add(8.0);
        optimalV0List.add(7.0);
        optimalV0List.add(8.5);
        optimalV0List.add(6.5);
        
        // 最优仰角列表（弧度）：根据实际测试结果调整
        optimalThetaList.add(Math.toRadians(30)); // 首选仰角 30 度
        optimalThetaList.add(Math.toRadians(25));
        optimalThetaList.add(Math.toRadians(35));
        optimalThetaList.add(Math.toRadians(20));
        optimalThetaList.add(Math.toRadians(40));
    }

    public AutoSelectResult Select(double relativeX, double relativeY, double robotVx, double robotVy, String mode) {
        switch (mode) {
            case "V":
                return selectByOptimalV0(relativeX, relativeY, robotVx, robotVy);
            case "Y":
                return selectByOptimalTheta(relativeX, relativeY, robotVx, robotVy);
            case "Pv":
                AutoSelectResult resultV = selectByOptimalV0(relativeX, relativeY, robotVx, robotVy);
                if (resultV.success) {
                    return resultV;
                }
                return selectByOptimalTheta(relativeX, relativeY, robotVx, robotVy);
            case "Py":
                AutoSelectResult resultY = selectByOptimalTheta(relativeX, relativeY, robotVx, robotVy);
                if (resultY.success) {
                    return resultY;
                }
                return selectByOptimalV0(relativeX, relativeY, robotVx, robotVy);
            default:
                return new AutoSelectResult(0, 0, false, "Invalid mode");
        }
    }

    private AutoSelectResult selectByOptimalV0(double relativeX, double relativeY, double robotVx, double robotVy) {
        for (double v0 : optimalV0List) {
            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, v0);
            if (result.success) {
                double theta = result.theta;
                double[] yawModeThetaRange = solver.getYawModeThetaRange();
                if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                    return new AutoSelectResult(theta, v0, true, "Success");
                }
            }
        }
        return new AutoSelectResult(0, 0, false, "No valid v0 found");
    }

    private AutoSelectResult selectByOptimalTheta(double relativeX, double relativeY, double robotVx, double robotVy) {
        for (double theta : optimalThetaList) {
            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, theta, "Vel");
            if (result.success) {
                double v0 = result.v0;
                if (v0 >= v0Min && v0 <= v0Max) {
                    return new AutoSelectResult(theta, v0, true, "Success");
                }
            }
        }
        return new AutoSelectResult(0, 0, false, "No valid theta found");
    }

    public void setOptimalV0List(List<Double> optimalV0List) {
        this.optimalV0List = optimalV0List;
    }

    public void setOptimalThetaList(List<Double> optimalThetaList) {
        this.optimalThetaList = optimalThetaList;
    }

    public void setV0Range(double v0Min, double v0Max) {
        this.v0Min = v0Min;
        this.v0Max = v0Max;
    }

    public void setThetaRange(double thetaMin, double thetaMax) {
        this.thetaMin = thetaMin;
        this.thetaMax = thetaMax;
    }

    public static class AutoSelectResult {
        public final double theta;
        public final double v0;
        public final boolean success;
        public final String message;

        public AutoSelectResult(double theta, double v0, boolean success, String message) {
            this.theta = theta;
            this.v0 = v0;
            this.success = success;
            this.message = message;
        }

        public double getThetaDegrees() {
            return Math.toDegrees(theta);
        }
    }
}
