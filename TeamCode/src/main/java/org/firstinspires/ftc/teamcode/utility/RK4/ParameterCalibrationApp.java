package org.firstinspires.ftc.teamcode.utility.RK4;

/// ========== [可删除] 参数标定应用程序 ==========
/// 此文件用于在计算机上运行参数标定（桌面应用，非 FTC OpMode）
/// 测定完成后可删除此文件
/// 相关代码：CalibrationHelper.java, ParameterCalculator.java, velocity_calibration.csv, drag_calibration.csv
/// =============================================

import java.io.File;
import java.util.Scanner;

public class ParameterCalibrationApp {

    public static void main(String[] args) {
        System.out.println("=====================================");
        System.out.println("RK4 跑打算法参数标定工具");
        System.out.println("=====================================");
        System.out.println("此工具从 CSV 文件读取实验数据并计算物理参数");
        System.out.println();
        
        Scanner scanner = new Scanner(System.in);
        String rk4Path = getRK4Path(scanner);
        
        ParameterCalculator calculator = new ParameterCalculator();
        
        // 设置基本参数
        ProjectileParameters params = new ProjectileParameters();
        params.deltaH = getDoubleInput(scanner, "请输入炮口高度 (米) [默认: 0.2]: ", 0.2);
        params.m = getDoubleInput(scanner, "请输入小球质量 (公斤) [默认: 0.1]: ", 0.1);
        calculator.setParameters(params);
        
        System.out.println();
        System.out.println("正在计算参数...");
        System.out.println("=====================================");
        
        ParameterCalculator.CalculationResult result = calculator.calculateFromCSV(rk4Path);
        
        System.out.println();
        System.out.println("===== 标定结果 =====");
        if (result.success) {
            System.out.printf("初速度 (v0): %.3f m/s\n", result.v0);
            System.out.printf("阻力系数 (k): %.6f\n", result.k);
            System.out.printf("速度指数 (n): %.3f\n", result.n);
            System.out.printf("拟合总误差: %.6f\n", result.totalError);
            System.out.printf("均方根误差: %.6f\n", result.getRmse());
            System.out.printf("初速度数据点: %d\n", result.velocityDataPoints);
            System.out.printf("阻力参数数据点: %d\n", result.dragDataPoints);
            System.out.println("状态: 成功");
            
            System.out.println();
            System.out.println("===== 推荐代码 =====");
            System.out.println("// 在比赛代码中使用以下参数:");
            System.out.printf("params.v0 = %.3f;\n", result.v0);
            System.out.printf("params.k = %.6f;\n", result.k);
            System.out.printf("params.n = %.3f;\n", result.n);
        } else {
            System.out.println("状态: 失败");
            System.out.println("错误: " + result.message);
        }
        
        System.out.println();
        System.out.println("=====================================");
        System.out.println("按 Enter 键退出...");
        scanner.nextLine();
        scanner.close();
    }
    
    private static String getRK4Path(Scanner scanner) {
        System.out.println("请输入 RK4 目录的路径:");
        System.out.println("示例: C:\\Users\\terry\\Documents\\trae_projects\\blue_power\\19725_DECODE_OffSeasonGame\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\utility\\RK4");
        System.out.println("按 Enter 使用当前目录:");
        
        String input = scanner.nextLine().trim();
        if (input.isEmpty()) {
            String currentDir = new File(".").getAbsolutePath();
            System.out.println("使用当前目录: " + currentDir);
            return currentDir;
        }
        
        File path = new File(input);
        if (!path.exists() || !path.isDirectory()) {
            System.out.println("错误: 目录不存在或不是有效目录");
            return getRK4Path(scanner);
        }
        
        System.out.println("使用目录: " + input);
        return input;
    }
    
    private static double getDoubleInput(Scanner scanner, String prompt, double defaultValue) {
        System.out.print(prompt);
        String input = scanner.nextLine().trim();
        if (input.isEmpty()) {
            System.out.println("使用默认值: " + defaultValue);
            return defaultValue;
        }
        
        try {
            double value = Double.parseDouble(input);
            return value;
        } catch (NumberFormatException e) {
            System.out.println("错误: 输入不是有效的数字");
            return getDoubleInput(scanner, prompt, defaultValue);
        }
    }
}