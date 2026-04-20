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



    public CalibrationData fitDragParameters(double[][] rangeAngleData,
                                              ProjectileParameters params) {
        double bestK = params.k;
        double bestN = params.n;
        double bestError = Double.MAX_VALUE;

        for (double n = 1.5; n <= 2.2; n += 0.1) {
            for (double k = 0.005; k <= 0.05; k += 0.005) {
                double error = calculateFitError(rangeAngleData, params.v0, k, n, params.m);
                if (error < bestError) {
                    bestError = error;
                    bestK = k;
                    bestN = n;
                }
            }
        }

        return new CalibrationData(bestK, bestN, bestError);
    }

    private double calculateFitError(double[][] rangeAngleData, double v0,
                                     double k, double n, double m) {
        double totalError = 0;
        ProjectileParameters testParams = new ProjectileParameters(v0, k, n, m, 0.5, Math.PI / 4);

        for (double[] data : rangeAngleData) {
            double targetAngle = data[0];
            double targetRange = data[1];

            TrajectorySimulator.TrajectoryResult result = simulator.simulate(
                0, targetAngle,
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