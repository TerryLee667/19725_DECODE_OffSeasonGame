package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.RK4.AutoSelect;
import org.firstinspires.ftc.teamcode.utility.RK4.Solver;
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretDegreeController;

import java.util.List;

public class Turret {
    // 核心组件
    private Shooter shooter;                   // 发射系统
    private TurretDegreeController turretDegreeController; // 炮台旋转控制器
    private AprilTagProcessor aprilTag;         // AprilTag处理器
    private VisionPortal visionPortal;          // 视觉门户
    private String team_color;                  // 团队颜色
    private int blueTagID;                     // 蓝队AprilTag ID
    private int redTagID;                      // 红队AprilTag ID

    
    // 电机和伺服
    private Servo launchServo;                 // 发射机构伺服电机
    
    // 状态变量
    private double roll;                        // 绕z轴角（水平旋转角）
    private double yaw;                         // 绕y轴角（仰角）
    private double k;                           // 速度转换参数k
    private double b;                           // 速度转换参数b
    private double delta_H;                      // 炮口与目标的高度差
    
    // 发射机构状态
    private static final double SERVO_REST_POSITION = 0.0;    // 伺服电机休息位置
    private static final double SERVO_LAUNCH_POSITION = 0.5;  // 伺服电机发射位置 
    
    // 常量
    private static final double APRILTAG_ANGLE_TOLERANCE = 0.5;      // AprilTag瞄准容差（度）
    
    // 构造函数
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, double k,double b,double delta_H, String teamColor, int blueTagId, int redTagId) {
        // 初始化发射系统
        shooter = new Shooter(hardwareMap, telemetry,k,b);
        
        // 初始化炮台旋转控制器
        turretDegreeController = new TurretDegreeController(hardwareMap);
        
        // 初始化发射机构伺服电机
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        launchServo.setPosition(SERVO_REST_POSITION);
        
        // 初始化AprilTag处理器
        initAprilTag(hardwareMap);
        
        // 初始化参数
        roll = 0.0;
        yaw = 0.0;
        this.k = k;
        this.b = b;
        this.delta_H = delta_H;
        this.team_color = teamColor;
        this.blueTagID = blueTagId;
        this.redTagID = redTagId;
    }
    
    /**
     * 简化构造函数，使用默认的tag ID
     */
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, double k,double b,double delta_H) {
        this(hardwareMap, telemetry, k, b, delta_H, "blue", 1, 2);
    }
    

    /**
     * 获取当前高度差
     * @return 当前高度差
     */
    public double getDeltaH() {
        return delta_H;
    }
    
    // 初始化AprilTag处理器
    private void initAprilTag(HardwareMap hardwareMap) {
        // 创建AprilTag处理器
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        
        // 创建视觉门户
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    
    /**
     * 旋转到指定角度
     * @param roll 水平旋转角（度，逆时针为正）
     * @param yaw 仰角（度）
     * @return 是否成功
     */
    public boolean rotate_to(double roll, double yaw){
        boolean success = turretDegreeController.rotateTo(roll, yaw);
        if (success) {
            // 旋转成功后更新内部状态
            this.roll = roll;
            this.yaw = yaw;
        }
        return success;
    }
    
    /**
     * 获取当前角度
     * @return 包含roll和yaw的数组 [roll, yaw]
     */
    public double[] get_angle() {
        // 使用TurretDegreeController获取当前角度
        double[] angles = turretDegreeController.get_angle();
        roll = angles[0];
        yaw = angles[1];
        return new double[]{roll, yaw};
    }
    

    
    /**
     * 使用AprilTag自动瞄准（非阻塞版本）
     * @return 包含检测状态和角度的数组：[isTargetFound, roll, yaw]
     */
    public Object[] aim() {
        // 先更新当前角度
        get_angle();
        
        // 获取AprilTag检测结果
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean isTargetFound = false;
        AprilTagDetection targetDetection = null;
        
        // 确定目标tag ID
        int targetTagId = team_color.equalsIgnoreCase("blue") ? blueTagID : redTagID;
        
        // 遍历检测结果，找到目标tag
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                targetDetection = detection;
                isTargetFound = true;
                break;
            }
        }
        
        if (isTargetFound && targetDetection != null) {
            // 使用找到的目标tag
            // 获取角度偏移
            double bearing = targetDetection.ftcPose.bearing; // 水平角度偏移
            double elevation = targetDetection.ftcPose.elevation; // 垂直角度偏移
            
            // 计算目标角度
            double targetRoll = roll + bearing;
            double targetYaw = yaw + elevation;
            
            // 旋转到目标角度
            rotate_to(targetRoll, targetYaw);
        } else {
            // 未检测到目标tag，水平逆时针旋转90度
            double targetRoll = roll + 90;
            rotate_to(targetRoll, yaw);
        }
        
        // 获取当前角度
        double[] angles = get_angle();
        // 返回包含检测状态和角度的数组
        return new Object[]{isTargetFound, angles[0], angles[1]};
    }
    
    /**
     * 设置速度转换参数
     * @param k 比例系数
     * @param b 截距
     */
    public void set(double k, double b) {
        this.k = k;
        this.b = b;
    }
    
    /**
     * 设置团队颜色
     * @param teamColor 团队颜色，"blue"或"red"
     */
    public void setTeamColor(String teamColor) {
        this.team_color = teamColor;
    }
    
    /**
     * 设置AprilTag ID
     * @param blueTagId 蓝队AprilTag ID
     * @param redTagId 红队AprilTag ID
     */
    public void setTagIDs(int blueTagId, int redTagId) {
        this.blueTagID = blueTagId;
        this.redTagID = redTagId;
    }
    
    /**
     * 发射小球
     * @param roll 目标旋转角
     * @param yaw 目标仰角
     */
    public void shoot(double roll, double yaw) {
        // 计算目标相对位置
        double deltaH = delta_H; // 使用成员变量delta_H
        double cotYaw = 1.0 / Math.tan(Math.toRadians(yaw));
        double targetX = deltaH * cotYaw * Math.cos(Math.toRadians(roll));
        double targetY = deltaH * cotYaw * Math.sin(Math.toRadians(roll));
        
        // 使用RK4计算发射参数
        AutoSelect autoSelect = new AutoSelect();
        // 设置高度差（炮口与目标的高度差）
        autoSelect.setDeltaH(deltaH);
        // 从Shooter类获取当前速度，并使用k和b转换为物理速度v0
        double currentSpeed = shooter.getCurrentVelocity();
        double initialV0 = (k != 0) ? (currentSpeed - b) / k : 8.0; // 使用k和b转换，k为0时使用默认值
        double initialTheta = Math.toRadians(yaw);
        AutoSelect.AutoSelectResult result = autoSelect.Select(targetX, targetY, 0, 0, initialV0, initialTheta);
        
        if (result.success) {
            // 将v0转换为转速
            double v0 = result.v0;
            int speed = (int) (k * v0 + b);
            
            // 控制发射电机达到目标速度
            boolean ready = false;
            while (!ready) {
                // 更新发射系统，确保电机获得功率
                shooter.update();
                ready = shooter.setTargetSpeed(speed);
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
            
            // 触发发射机构
            launchServo.setPosition(SERVO_LAUNCH_POSITION);
            try {
                Thread.sleep(300); // 等待发射动作完成
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            launchServo.setPosition(SERVO_REST_POSITION);
        }
    }
    
    /**
     * 每帧调用的更新函数
     * @param shouldShoot 是否发射
     */
    public void update(boolean shouldShoot) {
        // 更新发射系统
        shooter.update();
        
        // 瞄准目标
        Object[] aimResult = aim();
        boolean isTargetFound = (boolean) aimResult[0];
        double targetRoll = (double) aimResult[1];
        double targetYaw = (double) aimResult[2];
        
        // 如果需要发射且检测到目标
        if (shouldShoot && isTargetFound) {
            shoot(targetRoll, targetYaw);
        }
    }
    
    /**
     * 关闭视觉门户
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}