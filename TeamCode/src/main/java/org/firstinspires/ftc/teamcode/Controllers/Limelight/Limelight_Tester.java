package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import static org.firstinspires.ftc.teamcode.Controllers.Limelight.Limelight_Calculater.tx;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Limelight_Calculater;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.TurnTester_PID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight_Tester", group = "Tests")
public class Limelight_Tester extends LinearOpMode {
    Limelight_Calculater limelight_calculater;
    TurnTester_PID turnTesterPid;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (gamepad2.yWasPressed()) {
            turnTesterPid = new TurnTester_PID(hardwareMap);
            limelight_calculater.turning(true);
        }
    }


}
