package org.firstinspires.ftc.teamcode.Controllers.Chassis;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
@TeleOp(name="ChassisTester", group="Tests")
public class ChassisTester extends LinearOpMode {

    long lastNanoTime=0;


    @Override
    public void runOpMode()throws InterruptedException{
        ChassisController chassis=new ChassisController(hardwareMap,telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            chassis.update(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            if(gamepad1.xWasReleased())chassis.exchangeNoHeadMode();
            lastNanoTime=System.nanoTime();
            chassis.telemetry();
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}
