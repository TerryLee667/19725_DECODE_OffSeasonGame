package org.firstinspires.ftc.teamcode.rubbishbin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.ChassisController_ver1;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;

@TeleOp(name = "TurretAimTester", group = "Tests")
public class TurretAimTester extends LinearOpMode {
    private Turret turret;
    public ChassisController_ver1 chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis = new ChassisController_ver1(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry, 0.5, 0.0, 10.0);

        boolean isRunning = false;
        waitForStart();

        chassis.ChassisStop();
        chassis.ChassisInit();
        isRunning = true;

        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                chassis.SwitchHeadMode();
            }

            if (gamepad1.bWasPressed()) {
                isRunning = !isRunning;
                if (!isRunning) {
                    chassis.ChassisStop();
                    chassis.localization.Localization();
                    chassis.localization.ChassisVelocityTelemetry();
                    chassis.ChassisModeTelemetry();
                }
            }

            if (gamepad1.aWasPressed()) {
                chassis.localization.resetPosition();
            }

            if (isRunning) {
                chassis.GamepadCalculator(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                );
                chassis.ChassisMoving(
                        chassis.driveXTrans,
                        chassis.driveYTrans,
                        chassis.drivethetaTrans
                );
                chassis.localization.Localization();
                chassis.localization.ChassisVelocityTelemetry();
                chassis.ChassisModeTelemetry();
            }

            if (gamepad1.yWasPressed()) {
                double targetRoll = 45.0;
                double targetYaw = 60.0;
                turret.rotate_to(targetRoll, targetYaw);
            }

            if (gamepad1.left_bumper) {
                turret.update(true, true);
            } else if (gamepad1.right_bumper) {
                turret.update(false, true);
            } else {
                turret.update();
            }

            double[] angles = turret.get_angle();
            telemetry.addData("Turret Roll", angles[0]);
            telemetry.addData("Turret Yaw", angles[1]);
            telemetry.addData("isRunning", isRunning);
            telemetry.update();
        }
    }
}