package org.firstinspires.ftc.teamcode.utility.RK4;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class TestRK4 {

    // 内部类：模拟小球运动和落点计算
    static class BallSimulator {
        private TrajectorySimulator trajectorySimulator;
        private Random random;

        public BallSimulator() {
            this.trajectorySimulator = new TrajectorySimulator(0.01); // 使用与TrajectorySimulator相同的时间步长
            this.random = new Random();
        }

        // 生成随机的 k, n 参数（使用更合理的范围，确保小球可以飞到3m）
        public ProjectileParameters generateRandomParameters() {
            ProjectileParameters params = new ProjectileParameters();
            params.v0 = 10; // 使用10m/s初速度进行测试
            // 使用固定参数进行测试，便于调试
            params.k = 0.0004;  // 显著减小k值，减小空气阻力
            params.n = 2.0;   // 固定n值
            params.m = 0.06;
            params.deltaH = 1; // 目标高度与炮口高度相同
            return params;
        }

        // 生成实验数据：使用不同的初速度（收集数据时deltaH=0）
        // 注意：参数拟合算法设计为在固定45度仰角下通过不同初速度的数据来拟合k和n
        public double[][] generateExperimentData(ProjectileParameters params, int numPoints) {
            double[][] data = new double[numPoints][2];
            double[] v0Values = {4, 6, 8, 10, 12, 14, 16, 18, 20, 22};
            
            for (int i = 0; i < numPoints; i++) {
                // 使用不同的初速度，固定45度仰角
                double v0 = v0Values[i];
                ProjectileParameters tempParams = new ProjectileParameters();
                tempParams.v0 = v0;
                tempParams.k = params.k;
                tempParams.n = params.n;
                tempParams.m = params.m;
                tempParams.deltaH = 0; // 收集数据时deltaH=0
                
                // 计算45度仰角时的射程
                TrajectorySimulator.TrajectoryResult result = trajectorySimulator.simulate(
                    0, Math.toRadians(45), 0, 0, 0, 0, 0, tempParams
                );
                double distance = result.getHorizontalDistance();
                data[i][0] = distance;
                data[i][1] = 45; // 固定45度仰角
                System.out.printf("初速度: %.1f m/s, 距离: %.2f m, 仰角: 45.00 度\n", v0, distance);
            }

            return data;
        }

        // 找到最优仰角（使用更精确的搜索）
        private double findOptimalTheta(double targetDistance, ProjectileParameters params) {
            // 限制仰角搜索范围为45-65度，与AutoSelect中的仰角范围保持一致
            double low = Math.toRadians(45);
            double high = Math.toRadians(65);
            double bestTheta = 0;
            double bestError = Double.MAX_VALUE;

            // 首先找到最大射程对应的仰角
            double maxDistance = 0;
            double optimalAngleForMaxRange = 0;
            for (double theta = Math.toRadians(45); theta <= Math.toRadians(65); theta += Math.toRadians(1)) {
                TrajectorySimulator.TrajectoryResult result = trajectorySimulator.simulate(
                    0, theta, 0, 0, 0, 0, 0, params
                );
                double distance = result.getHorizontalDistance();
                if (distance > maxDistance) {
                    maxDistance = distance;
                    optimalAngleForMaxRange = theta;
                }
            }

            // 检查最大射程是否能达到目标距离
            if (maxDistance < targetDistance * 0.9) {
                // 最大射程也达不到目标距离，使用最大射程对应的仰角
                System.out.printf("警告: 目标距离 %.2fm 超出范围(最大%.2fm)，使用最大射程角度\n", targetDistance, maxDistance);
                return optimalAngleForMaxRange;
            }

            // 使用二分法搜索最优仰角
            for (int i = 0; i < 500; i++) {
                double mid = (low + high) / 2;
                TrajectorySimulator.TrajectoryResult result = trajectorySimulator.simulate(
                    0, mid, 0, 0, 0, 0, 0, params
                );

                double distance = result.getHorizontalDistance();
                double error = Math.abs(distance - targetDistance);

                if (error < bestError) {
                    bestError = error;
                    bestTheta = mid;
                }

                // 根据落点距离调整搜索范围
                if (distance < targetDistance) {
                    low = mid;
                } else {
                    high = mid;
                }
            }

            return bestTheta;
        }

        // 模拟小球落点
        public TrajectorySimulator.TrajectoryResult simulateLanding(double turretPhi, double theta, 
                                                                   double startX, double startY, 
                                                                   double robotVx, double robotVy, 
                                                                   ProjectileParameters params) {
            return trajectorySimulator.simulate(turretPhi, theta, startX, startY, 0, robotVx, robotVy, params);
        }

        // 模拟小球轨迹并打印详细信息
        public void simulateAndPrintTrajectory(double theta, ProjectileParameters params) {
            System.out.println("模拟轨迹，仰角: " + Math.toDegrees(theta) + " 度");
            System.out.println("参数: k=" + params.k + ", n=" + params.n + ", v0=" + params.v0 + ", deltaH=" + params.deltaH);
            
            ProjectileState state = computeInitialState(0, theta, 0, 0, 0, 0, 0, params);
            double targetZ = params.deltaH;
            
            System.out.println("初始状态: x=" + state.x + ", y=" + state.y + ", z=" + state.z);
            System.out.println("初始速度: vx=" + state.vx + ", vy=" + state.vy + ", vz=" + state.vz);
            
            int steps = 0;
            while (state.z >= targetZ - 0.5 && steps < 1000) {
                state = rk4Step(state, params);
                steps++;
                
                if (steps % 100 == 0) {
                    System.out.printf("Step %d: z=%.3f, vz=%.3f\n", steps, state.z, state.vz);
                }
            }
            
            System.out.println("最终状态: z=" + state.z + ", vz=" + state.vz);
            System.out.println();
        }

        // 计算初始状态
        private ProjectileState computeInitialState(double turretPhi, double theta, 
                                                  double startX, double startY, double startZ, 
                                                  double robotVx, double robotVy, 
                                                  ProjectileParameters params) {
            double vx0 = params.v0 * Math.cos(theta) * Math.cos(turretPhi) + robotVx;
            double vy0 = params.v0 * Math.cos(theta) * Math.sin(turretPhi) + robotVy;
            double vz0 = params.v0 * Math.sin(theta);

            return new ProjectileState(startX, startY, startZ, vx0, vy0, vz0, 0);
        }

        // RK4 步进
        private ProjectileState rk4Step(ProjectileState state, ProjectileParameters params) {
            double dt = 0.01; // 使用与TrajectorySimulator相同的时间步长
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

        // 添加导数
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
    }

    // 写入实验数据到 CSV 文件
    private static void writeExperimentDataToCSV(double[][] data, String filePath, double[] v0Values) throws IOException {
        try (FileWriter writer = new FileWriter(filePath)) {
            writer.write("v0,distance,angle\n");
            for (int i = 0; i < data.length; i++) {
                double v0 = v0Values[i];
                writer.write(v0 + "," + data[i][0] + "," + data[i][1] + "\n");
            }
        }
    }

    // 计算两个点之间的距离误差
    private static double calculateDistanceError(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    public static void main(String[] args) {
        try {
            System.out.println("=== RK4 跑打算法测试 ===");
            System.out.println();

            // 1. 初始化模拟器
            BallSimulator simulator = new BallSimulator();

            // 2. 生成随机参数
            ProjectileParameters trueParams = simulator.generateRandomParameters();
            System.out.println("真实参数:");
            System.out.println("k = " + trueParams.k);
            System.out.println("n = " + trueParams.n);
            System.out.println("v0 = " + trueParams.v0);
            System.out.println();

            // 3. 生成实验数据
            int numPoints = 10;
            double[] v0Values = {4, 6, 8, 10, 12, 14, 16, 18, 20, 22};
            double[][] experimentData = simulator.generateExperimentData(trueParams, numPoints);
            System.out.println("生成的实验数据:");
            for (double[] data : experimentData) {
                System.out.printf("距离: %.1f m, 仰角: %.2f 度\n", data[0], data[1]);
            }
            System.out.println();

            // 4. 写入实验数据到 CSV 文件
            String csvPath = "c:\\Users\\terry\\Documents\\trae_projects\\blue_power\\RK4\\org\\firstinspires\\ftc\\teamcode\\utility\\RK4\\drag_calibration.csv";
            writeExperimentDataToCSV(experimentData, csvPath, v0Values);
            System.out.println("实验数据已写入: " + csvPath);
            System.out.println();

            // 5. 使用 ParameterCalculator 拟合参数
            ParameterCalculator calculator = new ParameterCalculator();
            ProjectileParameters fitParams = new ProjectileParameters();
            fitParams.v0 = trueParams.v0; // 使用相同的初速度
            fitParams.deltaH = trueParams.deltaH;
            fitParams.m = trueParams.m;
            calculator.setParameters(fitParams);

            String basePath = "c:\\Users\\terry\\Documents\\trae_projects\\blue_power\\RK4\\org\\firstinspires\\ftc\\teamcode\\utility\\RK4";
            ParameterCalculator.CalculationResult fitResult = calculator.calculateFromCSV(basePath);

            System.out.println("拟合结果:");
            System.out.println("拟合 k = " + fitResult.k);
            System.out.println("拟合 n = " + fitResult.n);
            System.out.println("总误差 = " + fitResult.totalError);
            System.out.println("均方根误差 = " + fitResult.getRmse());
            System.out.println();

            // 6. 计算参数拟合误差
            double kError = Math.abs(fitResult.k - trueParams.k) / trueParams.k * 100;
            double nError = Math.abs(fitResult.n - trueParams.n) / trueParams.n * 100;
            System.out.println("参数拟合误差:");
            System.out.printf("k 误差: %.2f%%\n", kError);
            System.out.printf("n 误差: %.2f%%\n", nError);
            System.out.println();

            // 调试：直接测试Solver对各个角度的模拟
            System.out.println("调试：测试Solver模拟不同角度");
            TrajectorySimulator debugSim = new TrajectorySimulator(0.01); // 使用与TrajectorySimulator相同的时间步长
            System.out.printf("使用 trueParams: k=%.4f, n=%.4f, v0=%.2f, deltaH=%.2f\n",
                trueParams.k, trueParams.n, trueParams.v0, trueParams.deltaH);
            for (double thetaDeg : new double[]{45, 50, 55, 60, 65}) {
                double theta = Math.toRadians(thetaDeg);
                TrajectorySimulator.TrajectoryResult result = debugSim.simulate(
                    0, theta, 0, 0, 0, 0, 0, trueParams
                );
                System.out.printf("trueParams: theta=%.0f度: landing=(%.4f, %.4f), dist=%.4f, reached=%b\n",
                    thetaDeg, result.landingX, result.landingY, result.getHorizontalDistance(), result.reachedTargetHeight);
            }
            System.out.printf("使用 fitParams: k=%.4f, n=%.4f, v0=%.2f, deltaH=%.2f\n",
                fitParams.k, fitParams.n, fitParams.v0, fitParams.deltaH);
            for (double thetaDeg : new double[]{45, 50, 55, 60, 65}) {
                double theta = Math.toRadians(thetaDeg);
                TrajectorySimulator.TrajectoryResult result = debugSim.simulate(
                    0, theta, 0, 0, 0, 0, 0, fitParams
                );
                System.out.printf("fitParams: theta=%.0f度: landing=(%.4f, %.4f), dist=%.4f, reached=%b\n",
                    thetaDeg, result.landingX, result.landingY, result.getHorizontalDistance(), result.reachedTargetHeight);
            }
            System.out.println();

            System.out.println();

            // 7. 循环测试 AutoSelect 多次
            AutoSelect autoSelect = new AutoSelect(fitParams);
            Random random = new Random();
            int testCount = 30; // 测试次数
            double totalError = 0;
            double maxError = 0;
            double minError = Double.MAX_VALUE;
            
            System.out.println("=== 开始多组测试 (" + testCount + "次) ===");
            
            for (int testIndex = 0; testIndex < testCount; testIndex++) {
                // 生成随机目标位置和小车速度
                double targetX = 2.5 + random.nextDouble() * 3.0; // 2-6 米，均值3米
                double targetY = -1.0 + random.nextDouble() * 2.0; // -1 到 1 米
                double robotVx = -0.5 + random.nextDouble() * 1.0; // -0.5 到 0.5 m/s
                double robotVy = -0.5 + random.nextDouble() * 1.0; // -0.5 到 0.5 m/s

                System.out.println("\n测试 " + (testIndex + 1) + ":");
                System.out.printf("目标位置: (%.2f, %.2f) m\n", targetX, targetY);
                System.out.printf("小车速度: (%.2f, %.2f) m/s\n", robotVx, robotVy);
                
                // 测试合并模式（同时输入初始初速度和仰角）
                double combinedInitialV0 = 7.5; // 合并模式的初始初速度
                double combinedInitialTheta = Math.toRadians(52); // 合并模式的初始仰角
                AutoSelect.AutoSelectResult combinedResult = autoSelect.Select(targetX, targetY, robotVx, robotVy, combinedInitialV0, combinedInitialTheta);
                System.out.println("合并模式结果:");
                if (combinedResult.success) {
                    System.out.printf("仰角: %.2f 度\n", combinedResult.getThetaDegrees());
                    System.out.printf("初速度: %.2f m/s\n", combinedResult.v0);
                    System.out.printf("旋转角: %.2f 度\n", combinedResult.getTurretPhiDegrees());
                    System.out.println("消息: " + combinedResult.message);
                } else {
                    System.out.println("合并模式失败: " + combinedResult.message);
                }

                // 模拟小球落点并计算误差
                if (combinedResult.success) {
                    ProjectileParameters combinedParams = fitParams.copy();
                    combinedParams.v0 = combinedResult.v0;
                    TrajectorySimulator.TrajectoryResult combinedLanding = simulator.simulateLanding(
                        combinedResult.turretPhi, combinedResult.theta,
                        0, 0, robotVx, robotVy, combinedParams
                    );

                    double combinedError = calculateDistanceError(combinedLanding.landingX, combinedLanding.landingY, targetX, targetY);
                    System.out.printf("落点: (%.2f, %.2f) m, 误差: %.2f m\n", combinedLanding.landingX, combinedLanding.landingY, combinedError);
                    
                    // 统计误差
                    totalError += combinedError;
                    if (combinedError > maxError) maxError = combinedError;
                    if (combinedError < minError) minError = combinedError;
                }
            }
            
            // 输出统计结果
            System.out.println("\n=== 测试统计结果 ===");
            System.out.printf("测试次数: %d\n", testCount);
            System.out.printf("平均误差: %.2f m\n", totalError / testCount);
            System.out.printf("最大误差: %.2f m\n", maxError);
            System.out.printf("最小误差: %.2f m\n", minError);
            System.out.printf("误差稳定性: %s\n", (maxError - minError < 0.1) ? "良好" : "一般");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
