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
        this.params = new ProjectileParameters();
        this.params.thetaMax = Math.toRadians(65); // 设置最大仰角为65度
        this.solver = new Solver(this.params);
        this.optimalV0List = new ArrayList<>();
        this.optimalThetaList = new ArrayList<>();
        this.v0Min = 0.0; // 最小初速度
        this.v0Max = 10.0; // 最大初速度    
        this.thetaMin = Math.toRadians(45);
        this.thetaMax = Math.toRadians(65);
        
        // 初始化最优初速度列表和最优仰角列表
        initializeOptimalLists();
    }

    public AutoSelect(ProjectileParameters params) {
        params.thetaMax = Math.toRadians(65); // 确保最大仰角为65度
        this.solver = new Solver(params);
        this.params = params;
        this.optimalV0List = new ArrayList<>();
        this.optimalThetaList = new ArrayList<>();
        this.v0Min = 0.0; // 最小初速度
        this.v0Max = 10.0; // 最大初速度
        this.thetaMin = Math.toRadians(45);
        this.thetaMax = Math.toRadians(65);
        
        // 初始化最优初速度列表和最优仰角列表
        initializeOptimalLists();
    }

        private void initializeOptimalLists() {
        // 最优初速度列表：从较小值开始，逐步增大
        optimalV0List.add(8.0);
        optimalV0List.add(10.0);
        optimalV0List.add(12.0);
        optimalV0List.add(2.0);
        optimalV0List.add(4.0);
        optimalV0List.add(6.0);
        
        // 最优仰角列表（弧度）：首选55度（最大射程仰角）
        optimalThetaList.add(Math.toRadians(55));
        optimalThetaList.add(Math.toRadians(50));
        optimalThetaList.add(Math.toRadians(60));
        optimalThetaList.add(Math.toRadians(45));
        optimalThetaList.add(Math.toRadians(65));
    }

    // 合并模式：同时输入初始初速度和仰角，使用二分法同时优化
    public AutoSelectResult Select(double relativeX, double relativeY, double robotVx, double robotVy, double initialV0, double initialTheta) {
        // 首先尝试初始参数组合
        Solver.SolverResult vResult = solver.solve(relativeX, relativeY, robotVx, robotVy, initialV0);
        Solver.SolverResult yResult = solver.solve(relativeX, relativeY, robotVx, robotVy, initialTheta, "Vel");
        
        // 记录初始解
        AutoSelectResult initialResult = null;
        if (vResult.success) {
            double theta = vResult.theta;
            double turretPhi = vResult.turretPhi;
            double[] yawModeThetaRange = solver.getYawModeThetaRange();
            if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                initialResult = new AutoSelectResult(theta, initialV0, turretPhi, true, "Success with initial v0");
            }
        }
        
        if (yResult.success && (initialResult == null || yResult.v0 >= v0Min && yResult.v0 <= v0Max)) {
            double v0 = yResult.v0;
            double turretPhi = yResult.turretPhi;
            if (v0 >= v0Min && v0 <= v0Max) {
                initialResult = new AutoSelectResult(initialTheta, v0, turretPhi, true, "Success with initial theta");
            }
        }
        
        // 使用二分法同时优化初速度和仰角，即使初始解存在也要进一步优化
        double v0Low = Math.max(v0Min, initialV0 - 1.5); // 扩大搜索范围
        double v0High = Math.min(v0Max, initialV0 + 1.5);
        double thetaLow = Math.max(thetaMin, initialTheta - Math.toRadians(10)); // 10度范围
        double thetaHigh = Math.min(thetaMax, initialTheta + Math.toRadians(10));
        
        double bestV0 = initialV0;
        double bestTheta = initialTheta;
        double bestTurretPhi = 0;
        double bestError = Double.MAX_VALUE;
        boolean foundValid = false;
        
        // 网格搜索 + 二分法
        for (int i = 0; i < 50; i++) { // 增加迭代次数到50次
            double v0Mid = (v0Low + v0High) / 2;
            double thetaMid = (thetaLow + thetaHigh) / 2;
            
            // 先固定v0，优化theta
            Solver.SolverResult result1 = solver.solve(relativeX, relativeY, robotVx, robotVy, v0Mid);
            if (result1.success) {
                double theta = result1.theta;
                double turretPhi = result1.turretPhi;
                if (theta >= thetaMin && theta <= thetaMax) {
                    // 计算落点误差
                    ProjectileParameters tempParams = params.copy();
                    tempParams.v0 = v0Mid;
                    TrajectorySimulator simulator = new TrajectorySimulator(0.01);
                    TrajectorySimulator.TrajectoryResult landing = simulator.simulate(
                        turretPhi, theta, 0, 0, 0, robotVx, robotVy, tempParams
                    );
                    double distanceError = Math.sqrt(
                        Math.pow(landing.landingX - relativeX, 2) + 
                        Math.pow(landing.landingY - relativeY, 2)
                    );
                    
                    // 综合误差：落点误差 + 参数调整量
                    double error = distanceError + 0.01 * (Math.abs(v0Mid - initialV0) + Math.abs(theta - initialTheta));
                    
                    if (error < bestError) {
                        bestError = error;
                        bestV0 = v0Mid;
                        bestTheta = theta;
                        bestTurretPhi = turretPhi;
                        foundValid = true;
                    }
                    // 调整theta范围
                    if (theta < initialTheta) {
                        thetaLow = theta;
                    } else {
                        thetaHigh = theta;
                    }
                }
            }
            
            // 再固定theta，优化v0
            Solver.SolverResult result2 = solver.solve(relativeX, relativeY, robotVx, robotVy, thetaMid, "Vel");
            if (result2.success) {
                double v0 = result2.v0;
                double turretPhi = result2.turretPhi;
                if (v0 >= v0Min && v0 <= v0Max) {
                    // 计算落点误差
                    ProjectileParameters tempParams = params.copy();
                    tempParams.v0 = v0;
                    TrajectorySimulator simulator = new TrajectorySimulator(0.01);
                    TrajectorySimulator.TrajectoryResult landing = simulator.simulate(
                        turretPhi, thetaMid, 0, 0, 0, robotVx, robotVy, tempParams
                    );
                    double distanceError = Math.sqrt(
                        Math.pow(landing.landingX - relativeX, 2) + 
                        Math.pow(landing.landingY - relativeY, 2)
                    );
                    
                    // 综合误差：落点误差 + 参数调整量
                    double error = distanceError + 0.01 * (Math.abs(v0 - initialV0) + Math.abs(thetaMid - initialTheta));
                    
                    if (error < bestError) {
                        bestError = error;
                        bestV0 = v0;
                        bestTheta = thetaMid;
                        bestTurretPhi = turretPhi;
                        foundValid = true;
                    }
                    // 调整v0范围
                    if (v0 < initialV0) {
                        v0Low = v0;
                    } else {
                        v0High = v0;
                    }
                }
            }
            
            // 缩小搜索范围
            v0Low = Math.max(v0Min, bestV0 - 0.1);
            v0High = Math.min(v0Max, bestV0 + 0.1);
            thetaLow = Math.max(thetaMin, bestTheta - Math.toRadians(0.5));
            thetaHigh = Math.min(thetaMax, bestTheta + Math.toRadians(0.5));
        }
        
        if (foundValid) {
            // 验证最终解
            ProjectileParameters finalParams = params.copy();
            finalParams.v0 = bestV0;
            TrajectorySimulator simulator = new TrajectorySimulator(0.01);
            TrajectorySimulator.TrajectoryResult finalLanding = simulator.simulate(
                bestTurretPhi, bestTheta, 0, 0, 0, robotVx, robotVy, finalParams
            );
            double finalError = Math.sqrt(
                Math.pow(finalLanding.landingX - relativeX, 2) + 
                Math.pow(finalLanding.landingY - relativeY, 2)
            );
            
            if (finalError < 0.1) {
                return new AutoSelectResult(bestTheta, bestV0, bestTurretPhi, true, "Success with optimized parameters");
            }
        }
        
        // 如果优化失败，使用初始解
        if (initialResult != null) {
            return initialResult;
        }
        
        // 最后尝试备选方案
        return selectByInitialV0(relativeX, relativeY, robotVx, robotVy, initialV0);
    }

    private AutoSelectResult selectByOptimalV0(double relativeX, double relativeY, double robotVx, double robotVy) {
        for (double v0 : optimalV0List) {
            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, v0);
            System.out.printf("DEBUG selectByOptimalV0: v0=%.1f, success=%b, theta=%.2f deg, turretPhi=%.2f deg\n",
                v0, result.success, Math.toDegrees(result.theta), Math.toDegrees(result.turretPhi));
            if (result.success) {
                double theta = result.theta;
                double turretPhi = result.turretPhi;
                double[] yawModeThetaRange = solver.getYawModeThetaRange();
                System.out.printf("DEBUG: thetaRange=[%.2f, %.2f] deg\n",
                    Math.toDegrees(yawModeThetaRange[0]), Math.toDegrees(yawModeThetaRange[1]));
                
                // 检查仰角是否在范围内
                if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                    System.out.printf("DEBUG: theta %.2f deg IS in range, returning success\n", Math.toDegrees(theta));
                    return new AutoSelectResult(theta, v0, turretPhi, true, "Success");
                } else {
                    System.out.printf("DEBUG: theta %.2f deg is OUT of range, trying adjustment\n", Math.toDegrees(theta));
                }
                
                // 仰角超出范围，尝试微调
                AutoSelectResult adjustedResult = adjustV0(v0, theta, yawModeThetaRange, relativeX, relativeY, robotVx, robotVy);
                if (adjustedResult.success) {
                    System.out.printf("DEBUG: adjustment succeeded: theta=%.2f deg, v0=%.2f\n",
                        Math.toDegrees(adjustedResult.theta), adjustedResult.v0);
                    return adjustedResult;
                }
            }
        }
        System.out.println("DEBUG: no valid v0 found in list");
        return new AutoSelectResult(0, 0, 0, false, "No valid v0 found");
    }
    
    // 微调初速度以获得范围内的仰角（使用与selectByInitialV0一致的调整逻辑）
    private AutoSelectResult adjustV0(double initialV0, double initialTheta, double[] thetaRange,
                                       double relativeX, double relativeY, double robotVx, double robotVy) {
        double step = 0.1; // 调整步长（从0.5改为0.1，提高精度）
        double currentV0 = initialV0;
        boolean increasing = false; // 默认先尝试减小初速度

        for (int i = 0; i < 100; i++) { // 最多尝试100次（步长减小，增加迭代次数）
            currentV0 += step * (increasing ? 1 : -1); // 先从initialV0-step开始

            // 检查是否在范围内
            if (currentV0 < v0Min || currentV0 > v0Max) {
                if (increasing) {
                    // 增大到上限仍不成功，尝试减小
                    increasing = false;
                    currentV0 = initialV0 - step;
                } else {
                    // 减小到下限仍不成功，尝试增大
                    increasing = true;
                    currentV0 = initialV0 + step;
                }
            }

            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, currentV0);
            if (result.success) {
                double theta = result.theta;
                double turretPhi = result.turretPhi;

                if (theta >= thetaRange[0] && theta <= thetaRange[1]) {
                    return new AutoSelectResult(theta, currentV0, turretPhi, true, "Success after adjustment");
                }
            }
        }
        return new AutoSelectResult(0, 0, 0, false, "Adjustment failed");
    }

    private AutoSelectResult selectByOptimalTheta(double relativeX, double relativeY, double robotVx, double robotVy) {
        for (double theta : optimalThetaList) {
            double[] velModeThetaRange = solver.getVelModeThetaRange();
            if (theta < velModeThetaRange[0] || theta > velModeThetaRange[1]) {
                continue;
            }
            
            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, theta, "Vel");
            if (result.success) {
                double v0 = result.v0;
                double turretPhi = result.turretPhi;
                if (v0 >= v0Min && v0 <= v0Max) {
                    return new AutoSelectResult(theta, v0, turretPhi, true, "Success");
                }
                
                // v0超出范围，尝试微调
                AutoSelectResult adjustedResult = adjustThetaForV0(theta, v0, relativeX, relativeY, robotVx, robotVy);
                if (adjustedResult.success) {
                    return adjustedResult;
                }
            }
        }
        return new AutoSelectResult(0, 0, 0, false, "No valid theta found");
    }
    
    // 微调仰角以获得范围内的v0
    private AutoSelectResult adjustThetaForV0(double initialTheta, double initialV0,
                                              double relativeX, double relativeY, double robotVx, double robotVy) {
        double step = Math.toRadians(1); // 每次调整1度
        double currentTheta = initialTheta;
        boolean increasing = initialV0 < v0Min; // v0太小则增大仰角
        
        for (int i = 0; i < 20; i++) {
            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, currentTheta, "Vel");
            if (result.success) {
                double v0 = result.v0;
                double turretPhi = result.turretPhi;
                
                if (v0 >= v0Min && v0 <= v0Max) {
                    return new AutoSelectResult(currentTheta, v0, turretPhi, true, "Success after adjustment");
                }
                
                if (increasing) {
                    currentTheta += step;
                } else {
                    currentTheta -= step;
                }
                
                double[] velModeThetaRange = solver.getVelModeThetaRange();
                if (currentTheta < velModeThetaRange[0] || currentTheta > velModeThetaRange[1]) {
                    break;
                }
            } else {
                break;
            }
        }
        return new AutoSelectResult(0, 0, 0, false, "Adjustment failed");
    }
    
    // 新的V模式实现：接受初始初速度，使用二分法同时优化初速度和仰角
    private AutoSelectResult selectByInitialV0(double relativeX, double relativeY, double robotVx, double robotVy, double initialV0) {
        // 首先尝试初始初速度
        Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, initialV0);
        if (result.success) {
            double theta = result.theta;
            double turretPhi = result.turretPhi;
            double[] yawModeThetaRange = solver.getYawModeThetaRange();
            if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                return new AutoSelectResult(theta, initialV0, turretPhi, true, "Success with initial v0");
            }
        }
        
        // 使用二分法同时优化初速度和仰角
        double v0Low = Math.max(v0Min, initialV0 - 2.0); // 初始搜索范围
        double v0High = Math.min(v0Max, initialV0 + 2.0);
        double bestV0 = initialV0;
        double bestTheta = 0;
        double bestTurretPhi = 0;
        boolean foundValid = false;
        
        // 二分法搜索
        for (int i = 0; i < 50; i++) { // 最多50次迭代
            double v0Mid = (v0Low + v0High) / 2;
            result = solver.solve(relativeX, relativeY, robotVx, robotVy, v0Mid);
            
            if (result.success) {
                double theta = result.theta;
                double turretPhi = result.turretPhi;
                double[] yawModeThetaRange = solver.getYawModeThetaRange();
                
                if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                    // 找到有效解，进一步优化
                    bestV0 = v0Mid;
                    bestTheta = theta;
                    bestTurretPhi = turretPhi;
                    foundValid = true;
                    
                    // 缩小搜索范围，寻找更优解
                    v0Low = v0Mid - 0.5;
                    v0High = v0Mid + 0.5;
                } else if (theta < yawModeThetaRange[0]) {
                    // 仰角太小，需要增大初速度
                    v0Low = v0Mid;
                } else {
                    // 仰角太大，需要减小初速度
                    v0High = v0Mid;
                }
            } else {
                // 求解失败，尝试调整搜索范围
                double midV0 = (v0Low + v0High) / 2;
                if (midV0 < initialV0) {
                    v0Low = midV0 + 0.1;
                } else {
                    v0High = midV0 - 0.1;
                }
            }
        }
        
        if (foundValid) {
            return new AutoSelectResult(bestTheta, bestV0, bestTurretPhi, true, "Success with binary search");
        }
        
        // 如果二分法失败，尝试线性搜索作为备选
        double step = 0.1;
        double currentV0 = initialV0;
        boolean increasing = false;
        
        // 计算目标距离
        double targetDistance = Math.sqrt(relativeX * relativeX + relativeY * relativeY);
        if (targetDistance > 20.0) {
            increasing = true;
        }
        
        for (int i = 0; i < 100; i++) {
            currentV0 += step * (increasing ? 1 : -1);
            
            if (currentV0 < v0Min || currentV0 > v0Max) {
                if (increasing) {
                    increasing = false;
                    currentV0 = initialV0 - step;
                } else {
                    increasing = true;
                    currentV0 = initialV0 + step;
                }
            }
            
            result = solver.solve(relativeX, relativeY, robotVx, robotVy, currentV0);
            if (result.success) {
                double theta = result.theta;
                double turretPhi = result.turretPhi;
                double[] yawModeThetaRange = solver.getYawModeThetaRange();
                if (theta >= yawModeThetaRange[0] && theta <= yawModeThetaRange[1]) {
                    return new AutoSelectResult(theta, currentV0, turretPhi, true, "Success after linear adjustment");
                }
            }
        }
        
        return new AutoSelectResult(0, 0, 0, false, "No valid v0 found after adjustment");
    }
    
    // 新的Y模式实现：接受初始仰角，尝试并微调
    private AutoSelectResult selectByInitialTheta(double relativeX, double relativeY, double robotVx, double robotVy, double initialTheta) {
        // 首先尝试初始仰角
        Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy, initialTheta, "Vel");
        if (result.success) {
            double v0 = result.v0;
            double turretPhi = result.turretPhi;
            if (v0 >= v0Min && v0 <= v0Max) {
                return new AutoSelectResult(initialTheta, v0, turretPhi, true, "Success with initial theta");
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
                    return new AutoSelectResult(0, 0, 0, false, "No valid theta found within range");
                }
            }
            
            // 尝试当前仰角
            result = solver.solve(relativeX, relativeY, robotVx, robotVy, currentTheta, "Vel");
            if (result.success) {
                double v0 = result.v0;
                double turretPhi = result.turretPhi;
                if (v0 >= v0Min && v0 <= v0Max) {
                    return new AutoSelectResult(currentTheta, v0, turretPhi, true, "Success after adjustment");
                }
            }
        }
        
        return new AutoSelectResult(0, 0, 0, false, "No valid theta found after adjustment");
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
    
    // 设置高度差（炮口与目标的高度差）
    public void setDeltaH(double deltaH) {
        this.params.deltaH = deltaH;
    }

    public static class AutoSelectResult {
        public final double theta;
        public final double v0;
        public final double turretPhi;
        public final boolean success;
        public final String message;

        public AutoSelectResult(double theta, double v0, double turretPhi, boolean success, String message) {
            this.theta = theta;
            this.v0 = v0;
            this.turretPhi = turretPhi;
            this.success = success;
            this.message = message;
        }

        public double getThetaDegrees() {
            return Math.toDegrees(theta);
        }
        
        public double getTurretPhiDegrees() {
            return Math.toDegrees(turretPhi);
        }
    }
}
