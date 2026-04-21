package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.PIDController;

public class MotorPIDCore {
    public DcMotorEx motor;
    public Telemetry telemetry;
    public double degreePertick = 0;

    // PID参数
    public double k_p = 0.01;
    public double k_i = 0.4;
    public double k_d = 0.012;
    public double max_i = 1;

    // 高速PID参数
    public int PIDSwitchSpeed = 750;
    public double MinPower_1 = 0.1;
    public double k_p_1 = 0.01;
    public double k_i_1 = 0.2;
    public double k_d_1 = 0.05;
    public double max_i_1 = 1;
    public double MinPower_2 = 0.3;
    public double k_p_2 = 0.011;
    public double k_i_2 = 0.05;
    public double k_d_2 = 0.08;
    public double max_i_2 = 0.6;

    public static int SpeedTolerance = 35;

    // 运行时变量
    private PIDController pidController;
    private double Power = 0;
    private double current_speed = 0;
    private double current_time;
    private double previous_time;
    private double current_encoder = 0;
    private boolean isTurning = false;
    private double targetAngle = 0;

    // 构造方法
    public MotorPIDCore(HardwareMap hardwareMap, String motorName, boolean ifReverse) {
        initMotor(hardwareMap, motorName, ifReverse);
        pidController = new PIDController(k_p_1, k_i_1, k_d_1, max_i_1);
        previous_time = System.currentTimeMillis();
    }

    public MotorPIDCore(HardwareMap hardwareMap, Telemetry telemetry, String motorName, boolean ifReverse) {
        initMotor(hardwareMap, motorName, ifReverse);
        this.telemetry = telemetry;
        pidController = new PIDController(k_p, k_i, k_d, max_i);
        previous_time = System.currentTimeMillis();
    }

    private void initMotor(HardwareMap hardwareMap, String motorName, boolean ifReverse) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(ifReverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    // 设置PID参数
    public void setPID(double k_p, double k_i, double k_d, double max_i) {
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
        this.max_i = max_i;
        pidController.setPID(k_p, k_i, k_d);
        pidController.setMaxI(max_i);
    }

    // 设置目标速度
    public boolean setTargetSpeed(int targetSpeed) {
        if (targetSpeed < PIDSwitchSpeed) {
            pidController.setPID(k_p_1, k_i_1, k_d_1);
            pidController.setMaxI(max_i_1);
        } else {
            pidController.setPID(k_p_2, k_i_2, k_d_2);
            pidController.setMaxI(max_i_2);
        }

        if (targetSpeed == 0) {
            motor.setPower(0);
            pidController.reset();
            previous_time = System.currentTimeMillis();
            return true;
        }

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = motor.getCurrentPosition();
        current_speed = motor.getVelocity(AngleUnit.DEGREES);
        double dt = (current_time - previous_time) <= 0 ? 1 : (current_time - previous_time);

        Power = pidController.calculate(targetSpeed, current_speed, dt);

        if (targetSpeed > 0) {
            Power = Range.clip(Power, targetSpeed < PIDSwitchSpeed ? MinPower_1 : MinPower_2, 1);
        }

        motor.setPower(Power);
        previous_time = current_time;
        return Math.abs(targetSpeed - current_speed) < SpeedTolerance;
    }

    // 转到指定角度（带自锁）
    public boolean turn(double targetAngle, double currentAngle) {
        this.targetAngle = targetAngle;
        isTurning = true;

        if (Double.isNaN(targetAngle) || Math.abs(targetAngle) < 0.0001) {
            motor.setPower(0);
            pidController.reset();
            previous_time = System.currentTimeMillis();
            isTurning = false;
            return true;
        }

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = motor.getCurrentPosition();

        double angleError = targetAngle - currentAngle;
        double dt = (current_time - previous_time) <= 0 ? 1 : (current_time - previous_time);

        Power = pidController.calculate(0, angleError, dt);
        Power = Range.clip(Power, -1, 1);

        motor.setPower(Power);
        previous_time = current_time;

        boolean reachedTarget = Math.abs(angleError) < 1;
        return reachedTarget;
    }

    // 停止转向
    public void stopTurn() {
        motor.setPower(0);
        pidController.reset();
        isTurning = false;
    }

    // Getter方法
    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getCurrent_encoder() {
        return current_encoder;
    }

    public double getPower() {
        return Power;
    }

    public double getCurrent_speed() {
        return current_speed;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public boolean isTurning() {
        return isTurning;
    }

    public boolean isAtTarget() {
        return Math.abs(targetAngle - motor.getCurrentPosition() * degreePertick) < 1;
    }
}



