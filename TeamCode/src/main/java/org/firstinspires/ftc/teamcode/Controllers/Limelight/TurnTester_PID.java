package org.firstinspires.ftc.teamcode.Controllers.Limelight;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.Motor_pid;


@TeleOp
public class TurnTester_PID {
    HardwareMap hardwareMap;
    Motor_pid motor;
    public TurnTester_PID(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = new Motor_pid(hardwareMap, "motor", false);
    }
    public void targeting(int target) {
        motor.setTargetSpeed(target);

    }



}
