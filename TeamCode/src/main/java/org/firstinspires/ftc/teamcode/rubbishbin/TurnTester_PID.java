package org.firstinspires.ftc.teamcode.rubbishbin;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.Motor_pid;


@Config
public class TurnTester_PID {
    HardwareMap hardwareMap;
    Motor_pid motor;
    public TurnTester_PID(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        motor = new Motor_pid(hardwareMap, "motor", false);
    }
    public void targeting(int target) {
        motor.setTargetSpeed(target);

    }



}
