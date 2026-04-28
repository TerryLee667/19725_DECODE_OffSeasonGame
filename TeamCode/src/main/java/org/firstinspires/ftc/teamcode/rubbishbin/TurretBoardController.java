package org.firstinspires.ftc.teamcode.rubbishbin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.TurretAimingCalculator;

@Config
public class TurretBoardController {
    // 核心组件
    private TurretAimingCalculator aimingCalculator; // 瞄准角度计算器
    private Servo turretServo;                   // 炮台驱动舵机
    //public static double ANGLE_TOLERANCE = 0.5;      // 瞄准容差（度），小于该值则停止

    //预设值
    //todo参考实际使用舵机
    public static double max_pos=1.0;
    public static double min_pos=0;
    public static double Position=(max_pos+min_pos)/2;   //起始位置，可修改
    private final double targetHeight=1.0;//目标高度，即球框高度，预设值
    private double boardRotateLimit=180;//为挡板最大旋转角度（角度制）
    private double adjust=10;     //角度计算偏移参数（针对大角度域，而非舵机内置角度域）


    // 状态变量
    private double currentPosition = Position; // 舵机当前朝向角（舵机内置单位）
    private double targetPosition = Position;  // 舵机目标朝向角（舵机内置单位）


    /**
     * 构造方法：初始化炮台控制器
     * @param hardwareMap FTC硬件映射对象
     * @param turretServoName 炮台电机在配置文件中的名称

     */
    public TurretBoardController(HardwareMap hardwareMap, String turretServoName){
        // 初始化瞄准计算器
        aimingCalculator = new TurretAimingCalculator(0.0, 0.0); // 初始目标位置可后续更新


        // 初始化炮台电机
        turretServo = hardwareMap.get(Servo.class, turretServoName);
        initTurretServo();

        // 初始化炮台角度（以电机当前编码器位置为0度基准）
        resetTurretAngle();
    }

    /**
     * 初始化舵机参数（重置位置）
     */
    private void initTurretServo() {
        Position=(max_pos+min_pos)/2;
        turretServo.setPosition(Position);
        currentPosition = Position;
        targetPosition = Position;
    }

    /**
     * 重置舵机角度基准
     */
    public void resetTurretAngle() {
        Position=(max_pos+min_pos)/2;
        turretServo.setPosition(Position);
        currentPosition = Position;
        targetPosition = Position;
    }

    /**
     * 核心更新函数（主程序每一帧调用）
     * @param Theta 机器人方向角（度，逆时针为正，前进方向与X轴夹角）
     * @return 当前炮台相对机器人的朝向角（度，逆时针为正）
     */
    public double update(double Theta) {
        //TODO:下一次写完计算器，给这玩意的调用改掉。。。。
        // 计算炮台需要纵向旋转到的角度（targetTheta：最终转换为舵机中单位制）
        double targetTheta = aimingCalculator.calculateTurretAxisAngle(
                Theta, targetHeight
        );
//        if(targetTheta>boardRotateLimit/2){
//            targetTheta=boardRotateLimit/2;
//        }
//        if(targetTheta<-boardRotateLimit/2){
//            targetTheta=-boardRotateLimit/2;
//        }
        //所有目标角应该控制在角度域内，所以不希望用到这几行。
        // 将角度转换为舵机 position 并应用到舵机上
        rotateToAngle(targetTheta);

        // 返回当前舵机位置（舵机内置单位）
        return currentPosition;
    }

    /**
     * 将一个角度（度）转换为舵机 position 值（舵机的内置 [min_pos, max_pos] 单位）
     * 不做越界裁剪，仅计算理论目标 position（与原实现保持一致的映射关系）。
     * @param targetTheta 目标角度（度）
     * @return 计算得到的舵机 position（可能超出 [min_pos, max_pos]）
     */
    private double computePositionFromAngle(double targetTheta) {
        return ((max_pos - min_pos) * (targetTheta + adjust) / boardRotateLimit) + (max_pos + min_pos) / 2;
    }

    /**
     * 将计算得到的舵机 position 应用到物理舵机，并执行边界裁剪与当前/目标位置更新。
     * @param positionValue 要设置的舵机 position（可超出边界，内部会裁剪）
     */
    private void applyServoPosition(double positionValue) {
        // 将计算结果写入成员 Position（代表当前尝试的目标 position）
        Position = positionValue;

        // 边界裁剪
        if (Position <= min_pos) {
            Position = min_pos;
        }
        if (Position >= max_pos) {
            Position = max_pos;
        }

        // 将裁剪后的 position 应用到硬件
        turretServo.setPosition(Position);

        // 更新当前读取到的实际位置
        currentPosition = turretServo.getPosition();
    }

    /**
     * 对外的“根据角度旋转舵机”方法：先计算目标 position，保存 targetPosition（未裁剪的理论目标），然后应用到舵机。
     * @param targetTheta 目标角度（度）
     */
    public void rotateToAngle(double targetTheta) {
        double computed = computePositionFromAngle(targetTheta);
        // 保存理论目标（裁剪前），保持与原逻辑一致
        targetPosition = computed;
        // 应用到舵机（内部会裁剪并更新 currentPosition）
        applyServoPosition(computed);
    }
    /**
     * 读取当前挡板角度
     * @return 挡板的朝向角（舵机内置单位）
     */
    public double getCurrentPosition() {
        return currentPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
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
     * 获取当前瞄准误差（度）
     * @return 挡板当前朝向与目标朝向的夹角（度）
     */
    public double getAimError() {
        return Math.abs((((currentPosition-targetPosition)-(max_pos+min_pos)/2)*boardRotateLimit/(max_pos-min_pos))-adjust);
        //角度制转内置的逆运算
    }
}
