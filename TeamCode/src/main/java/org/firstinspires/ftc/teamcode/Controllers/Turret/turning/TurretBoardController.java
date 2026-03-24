package org.firstinspires.ftc.teamcode.Controllers.Turret.turning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptScanServo;
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
        Position=((max_pos-min_pos)*(targetTheta+adjust)/boardRotateLimit)+(max_pos+min_pos)/2;
        //将角度制目标(限制：-90°~90°)转换为舵机内置单位
        targetPosition=Position;
        if(Position<=min_pos) {
            Position=min_pos;
        }
        if(Position>=max_pos) {
            Position=max_pos;
        }
        turretServo.setPosition(Position);
        currentPosition=turretServo.getPosition();
        // 5. 返回当前舵机目标朝向角
        return Position;
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
