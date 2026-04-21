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
        this.v0Max = 25.0; // 增大最大初速度
        this.thetaMin = 0;
        this.thetaMax = Math.toRadians(55);
        
        // 初始化最优初速度列表和最优仰角列表
        initializeOptimalLists();
    }

    public AutoSelect(ProjectileParameters params) {
        this.solver = new Solver(params);
        this.params = params;
        this.optimalV0List = new ArrayList<>();
        this.optimalThetaList = new ArrayList<>();
        this.v0Min = 2.0;
        this.v0Max = 25.0; // 增大最大初速度
        this.thetaMin = 0;
        this.thetaMax = Math.toRadians(55);
        
        // 初始化最优初速度列表和最优仰角列表
        initializeOptimalLists();
    }

    private void initializeOptimalLists() {
        // 最优初速度列表：根据实际测试结果调整
        optimalV0List.add(15.0); // 首选初速度
        optimalV0List.add(18.0);
        optimalV0List.add(20.0);
        optimalV0List.add(12.0);
        optimalV0List.add(22.0);
        
        // 最优仰角列表（弧度）：根据实际测试结果调整，范围 0-55 度
        optimalThetaList.add(Math.toRadians(30)); // 首选仰角 30 度
        optimalThetaList.add(Math.toRadians(25));
        optimalThetaList.add(Math.toRadians(35));
        optimalThetaList.add(Math.toRadians(20));
        optimalThetaList.add(Math.toRadians(40));
        optimalThetaList.add(Math.toRadians(45));
        optimalThetaList.add(Math.toRadians(50));
        optimalThetaList.add(Math.toRadians(55));
    }

    public AutoSelectResult Select(double relativeX, double relativeY, double robotVx, double robotVy, String mode) {
        switch (mode) {
            case "V":
                // 原来的V模式，用于Pv内部调用
                return selectByOptimalV0(relativeX, relativeY, robotVx, robotVy);
            case "Y":
                // 原来的Y模式，用于Py内部调用
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
    
    // 新的V模式：接受初始初速度，尝试并微调
    public AutoSelectResult SelectWithInitialV0(double relativeX, double relativeY, double robotVx, double robotVy, double initialV0) {
        return selectByInitialV0(relativeX, relativeY, robotVx, robotVy, initialV0);
    }
    
    // 新的Y模式：接受初始仰角，尝试并微调
    public AutoSelectResult SelectWithInitialTheta(double relativeX, double relativeY, double robotVx, double robotVy, double initialTheta) {
        return selectByInitialTheta(relativeX, relativeY, robotVx, robotVy, initialTheta);
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
            // 检查仰角是否在 Vel 模式范围内（0-55度）
            double[] velModeThetaRange = solver.getVelModeThetaRange();
            if (theta < velModeThetaRange[0] || theta > velModeThetaRange[1]) {
                continue; // 跳过不在范围内的仰角
            }
            
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
    
    // 新的V模式实现：接受初始初速度，尝试并微调
    private AutoSelectResult selectByInitialV0(double relativeX, double relativeY, double robotVx, double robotVy, double initialV0) {
        // 首先尝试初始初速度
        Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, initialV0);
        if (result.success) {
            double theta = result.theta;
            double[] yawModeThetaRange = solver.getYawModeThetaRange();
            if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                return new AutoSelectResult(theta, initialV0, true, "Success with initial v0");
            }
        }
        
        // 确定调整方向
        double step = 0.5; // 调整步长
        double currentV0 = initialV0;
        boolean increasing = true;
        
        // 先尝试增大初速度
        for (int i = 0; i < 20; i++) { // 最多尝试20次
            currentV0 += step * (increasing ? 1 : -1);
            
            // 检查是否在范围内
            if (currentV0 < v0Min || currentV0 > v0Max) {
                if (increasing) {
                    // 增大到上限仍不成功，尝试减小
                    increasing = false;
                    currentV0 = initialV0 - step;
                } else {
                    // 减小到下限仍不成功，返回失败
                    return new AutoSelectResult(0, 0, false, "No valid v0 found within range");
                }
            }
            
            // 尝试当前初速度
            result = solver.solve(relativeX, relativeY, robotVx, robotVy, currentV0);
            if (result.success) {
                double theta = result.theta;
                double[] yawModeThetaRange = solver.getYawModeThetaRange();
                if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                    return new AutoSelectResult(theta, currentV0, true, "Success after adjustment");
                }
            }
        }
        
        return new AutoSelectResult(0, 0, false, "No valid v0 found after adjustment");
    }
    
    // 新的Y模式实现：接受初始仰角，尝试并微调
    private AutoSelectResult selectByInitialTheta(double relativeX, double relativeY, double robotVx, double robotVy, double initialTheta) {
        // 首先尝试初始仰角
        Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, initialTheta, "Vel");
        if (result.success) {
            double v0 = result.v0;
            if (v0 >= v0Min && v0 <= v0Max) {
                return new AutoSelectResult(initialTheta, v0, true, "Success with initial theta");
            }
        }
        
        // 确定调整方向
        double step = Math.toRadians(1); // 调整步长（1度）
        double currentTheta = initialTheta;
        boolean increasing = true;
        
        // 先尝试增大仰角
        for (int i = 0; i < 20; i++) { // 最多尝试20次
            currentTheta += step * (increasing ? 1 : -1);
            
            // 检查是否在范围内
            double[] velModeThetaRange = solver.getVelModeThetaRange();
            if (currentTheta < velModeThetaRange[0] || currentTheta > velModeThetaRange[1]) {
                if (increasing) {
                    // 增大到上限仍不成功，尝试减小
                    increasing = false;
                    currentTheta = initialTheta - step;
                } else {
                    // 减小到下限仍不成功，返回失败
                    return new AutoSelectResult(0, 0, false, "No valid theta found within range");
                }
            }
            
            // 尝试当前仰角
            result = solver.solve(relativeX, relativeY, robotVx, robotVy, currentTheta, "Vel");
            if (result.success) {
                double v0 = result.v0;
                if (v0 >= v0Min && v0 <= v0Max) {
                    return new AutoSelectResult(currentTheta, v0, true, "Success after adjustment");
                }
            }
        }
        
        return new AutoSelectResult(0, 0, false, "No valid theta found after adjustment");
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
