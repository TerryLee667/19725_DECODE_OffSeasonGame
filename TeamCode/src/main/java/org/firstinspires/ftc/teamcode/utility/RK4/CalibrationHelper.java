package org.firstinspires.ftc.teamcode.utility.RK4;

/// ========== [可删除] 参数标定辅助代码 ==========
/// 此文件包含参数标定功能，仅用于测定物理参数
/// 测定完成后可删除此文件
/// 相关代码：ParameterCalculator.java, ParameterCalibrationApp.java, velocity_calibration.csv, drag_calibration.csv
/// ==============================================

public class CalibrationHelper {

    private TrajectorySimulator simulator;

    public CalibrationHelper() {
        this.simulator = new TrajectorySimulator();
    }



    public CalibrationData fitDragParameters(double[][] v0RangeAngleData, ProjectileParameters params) {
        double bestKx = params.kx;
        double bestKy = params.ky;
        double bestError = Double.MAX_VALUE;

        // 第一阶段：粗略搜索整个参数空间
        for (double kx = -0.1; kx <= 0.1; kx += 0.005) { // kx范围：-0.1~0.1
            for (double ky = -0.5; ky <= 0.5; ky += 0.025) { // ky范围：-0.5~0.5
                double error = calculateFitError(v0RangeAngleData, kx, ky, params.m);
                if (error < bestError) {
                    bestError = error;
                    bestKx = kx;
                    bestKy = ky;
                }
            }
        }

        // 第二阶段：在最优区域附近精细搜索
        double coarseKx = bestKx;
        double coarseKy = bestKy;
        
        for (double kx = Math.max(-0.1, coarseKx - 0.01); kx <= Math.min(0.1, coarseKx + 0.01); kx += 0.001) { // 更细的kx步长
            for (double ky = Math.max(-0.5, coarseKy - 0.05); ky <= Math.min(0.5, coarseKy + 0.05); ky += 0.005) { // 更细的ky步长
                double error = calculateFitError(v0RangeAngleData, kx, ky, params.m);
                if (error < bestError) {
                    bestError = error;
                    bestKx = kx;
                    bestKy = ky;
                }
            }
        }

        // 由于CalibrationData只返回k，我们将bestKx作为k返回，bestKy需要在调用处单独处理
        params.ky = bestKy;
        return new CalibrationData(bestKx, bestError);
    }

    // 计算拟合误差
    private double calculateFitError(double[][] v0RangeAngleData, double kx, double ky, double m) {
        double totalError = 0;
        
        for (int i = 0; i < v0RangeAngleData.length; i++) {
            double v0 = v0RangeAngleData[i][0];
            double targetRange = v0RangeAngleData[i][1];
            double angle = v0RangeAngleData[i][2];
            
            // 创建测试参数
            ProjectileParameters testParams = new ProjectileParameters(0, kx, ky, m, 0, angle);
            testParams.v0 = v0;
            
            // 模拟轨迹
            TrajectorySimulator.TrajectoryResult result = simulator.simulate(
                0, angle,
                0, 0, 0,
                0, 0,
                testParams
            );
            
            double simRange = result.getHorizontalDistance();
            double error = Math.pow(simRange - targetRange, 2);
            totalError += error;
        }

        return totalError;
    }

    public static class CalibrationData {
        public final double k;
        public final double totalError;

        public CalibrationData(double k, double totalError) {
            this.k = k;
            this.totalError = totalError;
        }

        public double getRmse(int numDataPoints) {
            return Math.sqrt(totalError / numDataPoints);
        }
    }
}