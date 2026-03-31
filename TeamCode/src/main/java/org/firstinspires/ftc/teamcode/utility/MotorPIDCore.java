package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class MotorPIDCore {
    // 通用参数
    public double BlockPower = -0.3;
    public DcMotorEx motor;
    public Telemetry telemetry;
    public double degreePertick = 0;
    
    // PID参数（第一套，用于低速或通用）
    public double k_p = 0.01;
    public double k_i = 0.4;
    public double k_d = 0.012;
    public double max_i = 1;
    
    // 第二套PID参数（用于高速）
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
    
    // 速度容差（用于判断是否达到目标）
    public static int SpeedTolerance = 35;
    
    // 运行时变量
    private PIDController pidController;
    private double Power = 0;
    private double current_speed = 0;
    private double current_time;
    private double previous_time;
    private double current_encoder = 0;
    private double previous_encoder = 0;
    private double current_error;
    private double previous_error;
    
    // 用于turn方法的临时变量
    private TelemetryPacket packet = new TelemetryPacket();
    
    // 构造方法（无Telemetry，用于Motor_pid风格）
    public MotorPIDCore(HardwareMap hardwareMap, String motorName, boolean ifReverse) {
        initMotor(hardwareMap, motorName, ifReverse);
        pidController = new PIDController(k_p_1, k_i_1, k_d_1, max_i_1);
        previous_time = System.currentTimeMillis();
    }
    
    // 初始化方法（带Telemetry，用于Motor_PID风格）
    public void init(HardwareMap hardwareMap, Telemetry telemetry, String motorName, boolean ifReverse) {
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
        if (ifReverse)
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    
    // ========== 来自 Motor_PID 的方法 ==========
    public double getVelocity() {
        return motor.getVelocity();
    }
    
    /**
     * 基于角度的转向控制（来自Motor_PID）
     * @param targetSpeed 目标速度
     * @param targetAngle 目标角度
     * @param currentAngle 当前角度
     * @return 是否到达目标角度（误差<50°）
     */
    public boolean turn(double targetSpeed, double targetAngle, double currentAngle) {
        pidController.setPID(k_p, k_i, k_d);
        if (Double.isNaN(targetAngle) || Math.abs(targetAngle) < 0.0001) {
            motor.setPower(0);
            pidController.reset();
            previous_time = System.currentTimeMillis();
            return true;
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = motor.getCurrentPosition();
        current_speed = motor.getVelocity(AngleUnit.DEGREES);
        double dt = (current_time - previous_time);
        if (dt <= 0) dt = 1;
        Power = pidController.calculate(targetSpeed, current_speed, dt);
        if (targetSpeed > 0) {
            Power = Range.clip(Power, 0.001, 1);
        }
        motor.setPower(Power);
        previous_time = current_time;
        double angleError = targetAngle - currentAngle;
        return Math.abs(angleError) < 5;
    }
    
    public void block() {
        motor.setPower(BlockPower);
    }
    
    // ========== 来自 Motor_pid 的方法 ==========
    public void setPIDsettings(int PIDSwitchSpeed, double MinPower_1, double k_p_1, double k_i_1, double k_d_1, double max_i_1,
                               double MinPower_2, double k_p_2, double k_d_2, double k_i_2, double max_i_2) {
        this.PIDSwitchSpeed = PIDSwitchSpeed;
        this.MinPower_1 = MinPower_1;
        this.k_p_1 = k_p_1;
        this.k_i_1 = k_i_1;
        this.k_d_1 = k_d_1;
        this.max_i_1 = max_i_1;
        this.MinPower_2 = MinPower_2;
        this.k_p_2 = k_p_2;
        this.k_d_2 = k_d_2;
        this.k_i_2 = k_i_2;
        this.max_i_2 = max_i_2;
    }
    
    /**
     * 设置目标速度（来自Motor_pid）
     * @param targetSpeed 目标速度（RPM或其他单位，与getVelocity一致）
     * @return 是否达到目标速度（误差小于SpeedTolerance）
     */
    public boolean setTargetSpeed(int targetSpeed) {
        // 选择PID参数
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
        double dt = (current_time - previous_time);
        if (dt <= 0) dt = 1;
        
        Power = pidController.calculate(targetSpeed, current_speed, dt);
        
        if (targetSpeed > 0 && targetSpeed < PIDSwitchSpeed) {
            Power = Range.clip(Power, MinPower_1, 1);
        } else if (targetSpeed > 0) {
            Power = Range.clip(Power, MinPower_2, 1);
        }
        
        motor.setPower(Power);
        previous_time = current_time;
        return Math.abs(targetSpeed - current_speed) < SpeedTolerance;
    }
    
    // ========== 通用 getter ==========
    public double getCurrent_encoder() {
        return current_encoder;
    }
    
    public double getPower() {
        return Power;
    }
    
    public double getCurrent_speed() {
        return current_speed;
    }
}