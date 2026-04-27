package org.firstinspires.ftc.teamcode.Controllers.Turret.turner;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretDegreeController {
    // 电机实例
    private DcMotorEx rollMotor;    // 水平旋转电机
    private DcMotorEx yawMotor;     // 仰角电机
    
    // 电机参数
    public static double ROLL_MOTOR_TICKS_PER_REV = 28;   // 水平旋转电机每转脉冲数
    public static double YAW_MOTOR_TICKS_PER_REV = 28;    // 仰角电机每转脉冲数
    public static double ROLL_TICKS_PER_DEGREE = ROLL_MOTOR_TICKS_PER_REV / 360.0; // 水平旋转每度脉冲数
    public static double YAW_TICKS_PER_DEGREE = YAW_MOTOR_TICKS_PER_REV / 360.0;  // 仰角每度脉冲数
    public static double ROLL_MAX_POWER = 0.8;            // 水平旋转电机最大功率
    public static double YAW_MAX_POWER = 0.6;             // 仰角电机最大功率
    public static double ANGLE_TOLERANCE = 0.5;           // 角度容差（度）
    
    // 状态变量
    private double currentRoll = 0.0;   // 当前水平旋转角（度，逆时针为正）
    private double currentYaw = 0.0;    // 当前仰角（度）
    private double targetRoll = 0.0;    // 目标水平旋转角
    private double targetYaw = 0.0;     // 目标仰角
    
    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param rollMotorName 水平旋转电机名称
     * @param yawMotorName 仰角电机名称
     */
    public TurretDegreeController(HardwareMap hardwareMap, String rollMotorName, String yawMotorName) {
        // 初始化水平旋转电机
        rollMotor = hardwareMap.get(DcMotorEx.class, rollMotorName);
        initRollMotor();
        
        // 初始化仰角电机
        yawMotor = hardwareMap.get(DcMotorEx.class, yawMotorName);
        initYawMotor();
        
        // 初始化角度
        resetAngles();
    }
    
    /**
     * 默认构造函数，使用默认电机名称
     * @param hardwareMap 硬件映射
     */
    public TurretDegreeController(HardwareMap hardwareMap) {
        this(hardwareMap, "rollMotor", "yawMotor");
    }
    
    /**
     * 初始化水平旋转电机
     */
    private void initRollMotor() {
        rollMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rollMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rollMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rollMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }
    
    /**
     * 初始化仰角电机
     */
    private void initYawMotor() {
        yawMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        yawMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        yawMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }
    
    /**
     * 重置角度基准
     */
    public void resetAngles() {
        rollMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        currentRoll = 0.0;
        currentYaw = 0.0;
        targetRoll = 0.0;
        targetYaw = 0.0;
    }
    
    /**
     * 旋转到指定角度
     * @param roll 水平旋转角（度，逆时针为正）
     * @param yaw 仰角（度）
     * @return 是否成功
     */
    public boolean rotateTo(double roll, double yaw) {
        // 更新目标角度
        targetRoll = normalizeAngle(roll);
        targetYaw = yaw;
        
        // 控制水平旋转
        boolean rollReached = rotateRollTo(targetRoll);
        
        // 控制仰角
        boolean yawReached = rotateYawTo(targetYaw);
        
        // 更新当前角度
        updateCurrentAngles();
        
        // 返回是否两个角度都已达到目标
        return rollReached && yawReached;
    }
    
    /**
     * 旋转水平电机到目标角度
     * @param targetAngle 目标水平角度
     * @return 是否达到目标
     */
    private boolean rotateRollTo(double targetAngle) {
        // 计算目标编码器位置
        int targetTicks = (int) (targetAngle * ROLL_TICKS_PER_DEGREE);
        // 计算当前编码器位置
        int currentTicks = rollMotor.getCurrentPosition();
        // 计算角度误差
        double angleError = Math.abs(targetAngle - getRollAngle());
        
        // 误差小于容差时停止电机，否则以可控功率驱动
        if (angleError < ANGLE_TOLERANCE) {
            rollMotor.setPower(0.0);
            return true;
        } else {
            // 简单比例控制（P控制）
            double power = Math.min(ROLL_MAX_POWER, Math.max(-ROLL_MAX_POWER, angleError / 90.0 * ROLL_MAX_POWER));
            // 调整功率方向
            power = (targetTicks > currentTicks) ? power : -power;
            rollMotor.setPower(power);
            return false;
        }
    }
    
    /**
     * 旋转仰角电机到目标角度
     * @param targetAngle 目标仰角
     * @return 是否达到目标
     */
    private boolean rotateYawTo(double targetAngle) {
        // 计算目标编码器位置
        int targetTicks = (int) (targetAngle * YAW_TICKS_PER_DEGREE);
        // 计算当前编码器位置
        int currentTicks = yawMotor.getCurrentPosition();
        // 计算角度误差
        double angleError = Math.abs(targetAngle - getYawAngle());
        
        // 误差小于容差时停止电机，否则以可控功率驱动
        if (angleError < ANGLE_TOLERANCE) {
            yawMotor.setPower(0.0);
            return true;
        } else {
            // 简单比例控制（P控制）
            double power = Math.min(YAW_MAX_POWER, Math.max(-YAW_MAX_POWER, angleError / 45.0 * YAW_MAX_POWER));
            // 调整功率方向
            power = (targetTicks > currentTicks) ? power : -power;
            yawMotor.setPower(power);
            return false;
        }
    }
    
    /**
     * 获取当前角度
     * @return 包含roll和yaw的数组 [roll, yaw]
     */
    public double[] get_angle() {
        updateCurrentAngles();
        return new double[]{currentRoll, currentYaw};
    }
    
    /**
     * 更新当前角度
     */
    private void updateCurrentAngles() {
        currentRoll = getRollAngle();
        currentRoll = normalizeAngle(currentRoll);
        currentYaw = getYawAngle();
    }
    
    /**
     * 从编码器获取水平旋转角度
     * @return 水平旋转角度（度）
     */
    private double getRollAngle() {
        return rollMotor.getCurrentPosition() / ROLL_TICKS_PER_DEGREE;
    }
    
    /**
     * 从编码器获取仰角
     * @return 仰角（度）
     */
    private double getYawAngle() {
        return yawMotor.getCurrentPosition() / YAW_TICKS_PER_DEGREE;
    }
    
    /**
     * 角度归一化到[-180, 180]度范围
     * @param angle 原始角度（度）
     * @return 归一化后的角度（度）
     */
    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }
    
    /**
     * 获取当前瞄准误差
     * @return 包含水平和仰角误差的数组 [rollError, yawError]
     */
    public double[] getAimError() {
        updateCurrentAngles();
        double rollError = Math.abs(normalizeAngle(targetRoll - currentRoll));
        double yawError = Math.abs(targetYaw - currentYaw);
        return new double[]{rollError, yawError};
    }
}
