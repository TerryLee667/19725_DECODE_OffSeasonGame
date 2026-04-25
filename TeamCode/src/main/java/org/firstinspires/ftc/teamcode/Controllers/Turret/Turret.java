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
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretAimController;
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
    
    // 电机和伺服
    private DcMotorEx yawMotor;                // 仰角电机
    private Servo launchServo;                 // 发射机构伺服电机
    
    // 状态变量
    private double roll;                        // 绕z轴角（水平旋转角）
    private double yaw;                         // 绕y轴角（仰角）
    private double k;                           // 速度转换参数k
    private double b;                           // 速度转换参数b
    public double delta_H;                      // 炮口与目标的高度差
    
    // 发射机构状态
    private static final double SERVO_REST_POSITION = 0.0;    // 伺服电机休息位置
    private static final double SERVO_LAUNCH_POSITION = 0.5;  // 伺服电机发射位置 
    
    // 常量
    private static final double YAW_TICKS_PER_DEGREE = 28.0 / 360.0; // 仰角电机每度脉冲数
    private static final double YAW_MAX_POWER = 0.6;                // 仰角电机最大功率
    private static final double YAW_ANGLE_TOLERANCE = 0.5;          // 仰角容差（度）
    private static final double APRILTAG_ANGLE_TOLERANCE = 0.5;      // AprilTag瞄准容差（度）
    
    // 构造函数
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        // 初始化发射系统
        shooter = new Shooter(hardwareMap, telemetry);
        
        // 初始化炮台旋转控制器
        turretDegreeController = new TurretDegreeController();
        
        // 初始化仰角电机
        yawMotor = hardwareMap.get(DcMotorEx.class, "yawMotor");
        initYawMotor();
        
        // 初始化发射机构伺服电机
        launchServo = hardwareMap.get(Servo.class, "launchServo");
        launchServo.setPosition(SERVO_REST_POSITION);
        
        // 初始化AprilTag处理器
        initAprilTag(hardwareMap);
        
        // 初始化参数
        roll = 0.0;
        yaw = 0.0;
        k = 1.0;
        b = 0.0;
        delta_H = 0.5; // 默认高度差
    }
    
    // 初始化仰角电机
    private void initYawMotor() {
        yawMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        yawMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        yawMotor.setDirection(DcMotorEx.Direction.FORWARD);
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
        // 从编码器获取仰角，确保仰角值准确
        yaw = getYawAngle();
        return new double[]{roll, yaw};
    }
    
    /**
     * 从编码器获取仰角
     * @return 仰角（度）
     */
    private double getYawAngle() {
        return yawMotor.getCurrentPosition() / YAW_TICKS_PER_DEGREE;
    }
    
    /**
     * 使用AprilTag自动瞄准
     * @return 目标的仰角和旋转角 [roll, yaw]
     */
    public double[] aim() {
        boolean targetFound = false;
        double targetRoll = 0;
        double targetYaw = 0;
        
        while (!targetFound) {
            // 先更新当前角度
            get_angle();
            
            // 获取AprilTag检测结果
            List<AprilTagDetection> detections = aprilTag.getDetections();
            
            if (!detections.isEmpty()) {
                // 使用第一个检测到的AprilTag
                AprilTagDetection detection = detections.get(0);
                
                // 获取角度偏移
                double bearing = detection.ftcPose.bearing; // 水平角度偏移
                double elevation = detection.ftcPose.elevation; // 垂直角度偏移
                
                // 计算目标角度
                targetRoll = roll + bearing;
                targetYaw = yaw + elevation;
                
                // 旋转到目标角度
                rotate_to(targetRoll, targetYaw);
                
                // 检查角度偏移是否小于阈值
                if (Math.abs(bearing) < APRILTAG_ANGLE_TOLERANCE && Math.abs(elevation) < APRILTAG_ANGLE_TOLERANCE) {
                    targetFound = true;
                }
            } else {
                // 未检测到AprilTag，水平逆时针旋转90度
                targetRoll = roll + 90;
                rotate_to(targetRoll, yaw);
            }
            
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        
        // 获取当前角度
        double[] angles = get_angle();
        return angles;
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
        // 瞄准目标
        double[] targetAngles = aim();
        double targetRoll = targetAngles[0];
        double targetYaw = targetAngles[1];
        
        // 如果需要发射
        if (shouldShoot) {
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