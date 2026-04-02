package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//双飞轮控制系统
public class ShooterAction {
    //左右飞轮
    private final Shooter leftShooter;
    private final Shooter rightShooter;
    
    Telemetry telemetry;
    
    //构造函数
    public ShooterAction(HardwareMap hardwareMap, Telemetry telemetryrc, String leftMotorName, String rightMotorName, boolean leftReverse, boolean rightReverse) {
        this.telemetry = telemetryrc;
        //初始化左右飞轮
        leftShooter = new Shooter(hardwareMap, telemetryrc, leftMotorName, leftReverse);
        rightShooter = new Shooter(hardwareMap, telemetryrc, rightMotorName, rightReverse);
    }
    
    /**
     * 控制双飞轮达到目标速度
     * @param targetSpeed 目标速度
     * @return 是否两个飞轮都达到目标速度
     */
    public boolean shoot(int targetSpeed) {
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
    
    /**
     * 测试速度准确性
     * @param targetSpeed 目标速度
     * @param tolerance 速度公差
     * @return 是否达到目标速度
     */
    public boolean testSpeedAccuracy(int targetSpeed, double tolerance) {
        shoot(targetSpeed);
        double leftError = Math.abs(targetSpeed - getLeftSpeed());
        double rightError = Math.abs(targetSpeed - getRightSpeed());
        return leftError < tolerance && rightError < tolerance;
    }
    
    /**
     * 测试同步性
     * @param targetSpeed 目标速度
     * @param syncTolerance 同步公差
     * @param speedTolerance 速度公差
     * @return 是否同步且达到目标速度
     */
    public boolean testSynchronization(int targetSpeed, double syncTolerance, double speedTolerance) {
        shoot(targetSpeed);
        return isSynchronized(syncTolerance) && testSpeedAccuracy(targetSpeed, speedTolerance);
    }
    
    /**
     * 执行速度阶梯测试
     * @param speeds 速度数组
     * @param durationMs 每个速度持续时间
     */
    public void runSpeedTest(int[] speeds, long durationMs) throws InterruptedException {
        for (int speed : speeds) {
            telemetry.addData("Testing Speed", speed);
            telemetry.update();
            
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < durationMs) {
                shoot(speed);
                setTelemetry();
                telemetry.update();
                Thread.sleep(50);
            }
        }
        stop();
    }
}