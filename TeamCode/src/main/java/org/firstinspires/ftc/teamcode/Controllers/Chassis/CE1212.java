package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="CE1212", group="Test")
public class CE1212 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ChassisController23.Constants constants = ChassisController23.Constants.DEFAULT();
        ChassisController23 chassis = new ChassisController23(hardwareMap, constants);

        telemetry.addData("status", "Initialized");
        telemetry.update();

        waitForStart();

        // Reset pose and encoders
        chassis.resetEncoders();
        chassis.setPose(0,0,0);

        // Example: fix the field X axis to point PI/2 radians CCW from robot's initial forward
        // (so fixed X points to robot's left). Change or remove as needed.
        chassis.setAxisAlignment(0.0);
        telemetry.addData("axisAlignmentRad", chassis.getAxisAlignment()); telemetry.update();

        // 1) forward 0.5m
        telemetry.addData("step", "forward 0.5m"); telemetry.update();
        chassis.moveRobotRelativeMeters(0.5, 0.0, 0.5);
        chassis.updateFromEncoders();
        telemetry.addData("pose_after_forward", chassis.getPose().toString()); telemetry.update();
        sleep(500);

        // 2) strafe right 0.5m
        telemetry.addData("step", "strafe right 0.5m"); telemetry.update();
        chassis.moveRobotRelativeMeters(0.0, 0.5, 0.5);
        chassis.updateFromEncoders();
        telemetry.addData("pose_after_strafe", chassis.getPose().toString()); telemetry.update();
        sleep(500);

        // 3) rotate approx 90deg in place by commanding differential wheel distances
        telemetry.addData("step", "rotate ~90deg"); telemetry.update();
        // approximate arc length for rotation: theta * (track/2) on each side
        double theta = Math.PI/2.0;
        double arc = theta * (constants.TRACK_WIDTH_M/2.0);
        // left moves backward, right moves forward
        chassis.moveRobotRelativeMeters(-arc, 0.0, 0.4); // move forward/back used as rotation approximation
        chassis.updateFromEncoders();
        telemetry.addData("pose_after_rotate", chassis.getPose().toString()); telemetry.update();
        sleep(500);

        telemetry.addData("done", "tests complete"); telemetry.update();
        while (opModeIsActive()) { idle(); }
    }
}
