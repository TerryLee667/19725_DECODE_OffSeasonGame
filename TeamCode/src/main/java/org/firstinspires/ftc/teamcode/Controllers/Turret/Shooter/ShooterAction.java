package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//双飞轮控制系统
public class ShooterAction {
    //左右飞轮
    private final Shooter1 leftShooter;
    private final Shooter1 rightShooter;
    
    Telemetry telemetry;
    private int targetSpeed=0;
    
    //构造函数
    public ShooterAction(HardwareMap hardwareMap, Telemetry telemetryrc, String leftMotorName, String rightMotorName, boolean leftReverse, boolean rightReverse) {
        this.telemetry = telemetryrc;
        //初始化左右飞轮
        //TODO 硬件配置（电机名称和反转设置）是确定的，建议写死在controller里，而不是从外部传入
        leftShooter = new Shooter1(hardwareMap, telemetryrc, leftMotorName, leftReverse);
        //TODO 考虑是用两个独立的PID控制器，还是一个用PID计算，另一个直接复制负的功率（如果电机性能一致，后者可能更稳定）
        rightShooter = new Shooter1(hardwareMap, telemetryrc, rightMotorName, rightReverse);
    }
    
    /**
     * 控制双飞轮达到目标速度
     * @param targetspeed 目标速度
     */
    public void shoot(int targetspeed) {
        this.targetSpeed=targetspeed;
    }

    /**
     * 每一帧必须调用此函数。
     * @return 是否两个飞轮都达到目标速度
     */
    public boolean update(){
        boolean leftReady = leftShooter.shoot(targetSpeed);
        boolean rightReady = rightShooter.shoot(targetSpeed);
        return leftReady && rightReady;
    }
    
    /**
     * 停止双飞轮
     */
    public void stop() {
        leftShooter.shoot(0);
        rightShooter.shoot(0);
    }
    
    /**
     * 阻塞双飞轮（反转）
     */
    public void block() {
        leftShooter.block();
        rightShooter.block();
    }
    
    /**
     * 获取左轮功率
     * @return 左轮功率
     */
    public double getLeftPower() {
        return leftShooter.getPower();
    }
    
    /**
     * 获取右轮功率
     * @return 右轮功率
     */
    public double getRightPower() {
        return rightShooter.getPower();
    }
    
    /**
     * 获取左轮速度
     * @return 左轮速度
     */
    public double getLeftSpeed() {
        return leftShooter.getCurrent_speed();
    }
    
    /**
     * 获取右轮速度
     * @return 右轮速度
     */
    public double getRightSpeed() {
        return rightShooter.getCurrent_speed();
    }
    
    /**
     * 获取速度差
     * @return 左右轮速度差
     */
    public double getSpeedDifference() {
        return Math.abs(getLeftSpeed() - getRightSpeed());
    }
    
    /**
     * 检查双飞轮是否同步
     * @param tolerance 速度公差
     * @return 是否同步
     */
    public boolean isSynchronized(double tolerance) {
        return getSpeedDifference() < tolerance;
    }
    
    /**
     * 设置遥测数据
     */
    public void setTelemetry() {
        telemetry.addData("Left Shooter Power", getLeftPower());
        telemetry.addData("Right Shooter Power", getRightPower());
        telemetry.addData("Left Shooter Velocity", getLeftSpeed());
        telemetry.addData("Right Shooter Velocity", getRightSpeed());
        telemetry.addData("Speed Difference", getSpeedDifference());
        telemetry.addData("Synchronized", isSynchronized(10)); // 10度/秒的公差
    }

}