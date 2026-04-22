package org.firstinspires.ftc.teamcode.utility.RK4;

/// ========== [可删除] 参数计算器 ==========
/// 此文件包含参数标定功能，仅用于测定物理参数
/// 测定完成后可删除此文件
/// 相关代码：CalibrationHelper.java, ParameterCalibrationApp.java, velocity_calibration.csv, drag_calibration.csv
/// ==========================================

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class ParameterCalculator {

    private CalibrationHelper calibrator;
    private ProjectileParameters params;

    public ParameterCalculator() {
        this.calibrator = new CalibrationHelper();
        this.params = new ProjectileParameters();
    }

    public ParameterCalculator(ProjectileParameters params) {
        this.calibrator = new CalibrationHelper();
        this.params = params;
    }

    public CalculationResult calculateFromCSV(String basePath) {
        try {
            // 读取阻力参数标定数据（包含v0）
            double[][] v0RangeAngleData = readV0RangeAngleData(basePath + "/drag_calibration.csv");
            
            // 拟合阻力参数（使用CSV中的v0数据）
            CalibrationHelper.CalibrationData dragResult = calibrator.fitDragParameters(v0RangeAngleData, params);
            
            // 更新参数
            params.k = dragResult.k;
            params.n = dragResult.n;
            
            return new CalculationResult(
                params.v0,
                dragResult.k,
                dragResult.n,
                dragResult.totalError,
                0, // 不再使用初速度标定数据
                v0RangeAngleData.length,
                true,
                "Successfully calculated parameters from CSV files"
            );
        } catch (Exception e) {
            return new CalculationResult(
                params.v0,
                0,
                0,
                Double.MAX_VALUE,
                0,
                0,
                false,
                "Error: " + e.getMessage()
            );
        }
    }

    private double[][] readV0RangeAngleData(String filePath) throws IOException {
        List<double[]> dataList = new ArrayList<>();
        Path path = Paths.get(filePath);
        
        if (!Files.exists(path)) {
            throw new IOException("File not found: " + filePath);
        }
        
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean isHeader = true;
            
            while ((line = br.readLine()) != null) {
                if (isHeader) {
                    isHeader = false;
                    continue;
                }
                
                line = line.trim();
                if (!line.isEmpty()) {
                    String[] parts = line.split(",");
                    if (parts.length >= 3) {
                        try {
                            double v0 = Double.parseDouble(parts[0].trim());
                            double range = Double.parseDouble(parts[1].trim());
                            double angle = Math.toRadians(Double.parseDouble(parts[2].trim()));
                            dataList.add(new double[]{v0, range, angle});
                        } catch (NumberFormatException e) {
                            // 跳过无效数据
                        }
                    }
                }
            }
        }
        
        if (dataList.isEmpty()) {
            throw new IOException("No valid v0-range-angle data found in file: " + filePath);
        }
        
        double[][] result = new double[dataList.size()][3];
        for (int i = 0; i < dataList.size(); i++) {
            result[i] = dataList.get(i);
        }
        
        return result;
    }
    
    private double[][] readRangeAngleData(String filePath) throws IOException {
        List<double[]> dataList = new ArrayList<>();
        Path path = Paths.get(filePath);
        
        if (!Files.exists(path)) {
            throw new IOException("File not found: " + filePath);
        }
        
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean isHeader = true;
            
            while ((line = br.readLine()) != null) {
                if (isHeader) {
                    isHeader = false;
                    continue;
                }
                
                line = line.trim();
                if (!line.isEmpty()) {
                    String[] parts = line.split(",");
                    if (parts.length >= 2) {
                        try {
                            double range = Double.parseDouble(parts[0].trim());
                            double angle = Math.toRadians(Double.parseDouble(parts[1].trim()));
                            dataList.add(new double[]{range, angle});
                        } catch (NumberFormatException e) {
                            // 跳过无效数据
                        }
                    }
                }
            }
        }
        
        if (dataList.isEmpty()) {
            throw new IOException("No valid range-angle data found in file: " + filePath);
        }
        
        double[][] result = new double[dataList.size()][2];
        for (int i = 0; i < dataList.size(); i++) {
            result[i] = dataList.get(i);
        }
        
        return result;
    }

    public ProjectileParameters getParameters() {
        return params;
    }

    public void setParameters(ProjectileParameters params) {
        this.params = params;
    }

    public static class CalculationResult {
        public final double v0;
        public final double k;
        public final double n;
        public final double totalError;
        public final int velocityDataPoints;
        public final int dragDataPoints;
        public final boolean success;
        public final String message;

        public CalculationResult(double v0, double k, double n, double totalError,
                                int velocityDataPoints, int dragDataPoints,
                                boolean success, String message) {
            this.v0 = v0;
            this.k = k;
            this.n = n;
            this.totalError = totalError;
            this.velocityDataPoints = velocityDataPoints;
            this.dragDataPoints = dragDataPoints;
            this.success = success;
            this.message = message;
        }

        public double getRmse() {
            if (dragDataPoints == 0) {
                return Double.MAX_VALUE;
            }
            return Math.sqrt(totalError / dragDataPoints);
        }

        @Override
        public String toString() {
            if (!success) {
                return "Calculation failed: " + message;
            }
            
            return String.format(
                "Parameter Calculation Result:\n" +
                "- Initial Velocity (v0): %.3f m/s\n" +
                "- Drag Coefficient (k): %.6f\n" +
                "- Velocity Exponent (n): %.3f\n" +
                "- Total Fit Error: %.6f\n" +
                "- RMSE: %.6f\n" +
                "- Velocity Data Points: %d\n" +
                "- Drag Data Points: %d\n" +
                "- Status: %s",
                v0, k, n, totalError, getRmse(),
                velocityDataPoints, dragDataPoints, message
            );
        }
    }

    public static void main(String[] args) {
        if (args.length < 1) {
            System.out.println("Usage: java ParameterCalculator <base_path>");
            System.out.println("Example: java ParameterCalculator ./data");
            return;
        }
        
        String basePath = args[0];
        ParameterCalculator calculator = new ParameterCalculator();
        
                // 设置基本参数（包括手动设置的初速度）
        ProjectileParameters params = new ProjectileParameters();
        params.v0 = 10.0;     // 手动设置的初速度（米/秒）
        params.deltaH = 0.2; // 炮口高度（米）
        params.m = 0.1;      // 小球质量（公斤）
        calculator.setParameters(params);
        
        CalculationResult result = calculator.calculateFromCSV(basePath);
        System.out.println(result);
        
        if (result.success) {
            // 保存参数到配置文件
            System.out.println("\nRecommended parameters for ProjectileParameters:");
            System.out.println("params.v0 = " + result.v0 + "; // 手动设置的初速度");
            System.out.println("params.k = " + result.k + ";");
            System.out.println("params.n = " + result.n + ";");
        }
    }
}