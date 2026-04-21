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



    public CalibrationData fitDragParameters(double[][] rangeAngleData, ProjectileParameters params) {
        double bestK = params.k;
        double bestN = params.n;
        double bestError = Double.MAX_VALUE;

        // 第一阶段：粗略搜索整个参数空间（扩大范围，包括更小的k值）
        for (double n = 1.0; n <= 3.0; n += 0.1) {
            for (double k = 0.0001; k <= 0.01; k += 0.0001) { // 更细的k步长，包括更小的值
                double error = calculateFitError(rangeAngleData, k, n, params.m);
                if (error < bestError) {
                    bestError = error;
                    bestK = k;
                    bestN = n;
                }
            }
        }

        // 第二阶段：在最优区域附近精细搜索
        double coarseK = bestK;
        double coarseN = bestN;
        
        for (double n = coarseN - 0.2; n <= coarseN + 0.2; n += 0.01) { // 更细的n步长
            if (n < 0.5 || n > 4.0) continue;
            for (double k = Math.max(0.00005, coarseK - 0.001); k <= Math.min(0.02, coarseK + 0.001); k += 0.00005) { // 更细的k步长
                double error = calculateFitError(rangeAngleData, k, n, params.m);
                if (error < bestError) {
                    bestError = error;
                    bestK = k;
                    bestN = n;
                }
            }
        }

        return new CalibrationData(bestK, bestN, bestError);
    }

    // 生成不同初速度的数据点
    private double[][] generateV0RangeData(double[] v0Values, double k, double n, double m) {
        double[][] data = new double[v0Values.length][2];
        ProjectileParameters testParams = new ProjectileParameters(0, k, n, m, 0, Math.PI / 4);

        for (int i = 0; i < v0Values.length; i++) {
            testParams.v0 = v0Values[i];
            TrajectorySimulator.TrajectoryResult result = simulator.simulate(
                0, Math.toRadians(45),
                0, 0, 0,
                0, 0,
                testParams
            );
            data[i][0] = result.getHorizontalDistance();
            data[i][1] = 45; // 45度仰角
        }

        return data;
    }

    private double calculateFitError(double[][] rangeAngleData, double k, double n, double m) {
        double totalError = 0;
        // 我们的实验数据是不同初速度下的45度仰角射程
        double[] v0Values = {4, 6, 8, 10, 12, 14, 16, 18, 20, 22};
        double[][] generatedData = generateV0RangeData(v0Values, k, n, m);

        for (int i = 0; i < rangeAngleData.length && i < generatedData.length; i++) {
            double targetRange = rangeAngleData[i][0];
            double simRange = generatedData[i][0];
            double error = Math.pow(simRange - targetRange, 2);
            totalError += error;
        }

        return totalError;
    }

    public static class CalibrationData {
        public final double k;
        public final double n;
        public final double totalError;

        public CalibrationData(double k, double n, double totalError) {
            this.k = k;
            this.n = n;
            this.totalError = totalError;
        }

        public double getRmse(int numDataPoints) {
            return Math.sqrt(totalError / numDataPoints);
        }
    }
}