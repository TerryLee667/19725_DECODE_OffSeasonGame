package org.firstinspires.ftc.teamcode.rubbishbin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class motor1 {
    public DcMotorEx motor;
    public double power = 30;
//    public double StopPower = 0;
//    public static double k_p = 0.01;
//    public static double k_i = 0.4;
//    public static double k_d = 0.012;
//    public static double max_i = 1;
//    private PIDController pidController;
    public Telemetry telemetry;
//    public double current_time;
//    public double previous_time;
//    public double current_encoder = 0;
//    public double current_speed = 0;
    public void Motor(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName, boolean ifReverse) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(ifReverse)
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetryrc;
//        pidController = new PIDController(k_p, k_i, k_d, max_i);
//        previous_time = System.currentTimeMillis();
    }

    public void motor_1() {
        motor.setPower(power);

    }
    public void motor_2() {
        motor.setPower(-power);
    }
    public void motor_3() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void motortelemetry(){
        if(this.telemetry!=null){
            this.telemetry.addData("motor power", motor.getPower());
            this.telemetry.addData("motor direction", motor.getDirection());
            this.telemetry.addData("motor current position", motor.getCurrentPosition());
            this.telemetry.update();
        }
    }
}
