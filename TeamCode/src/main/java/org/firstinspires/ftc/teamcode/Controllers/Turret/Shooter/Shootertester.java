package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Shootertester", group = "Tests")
public class Shootertester extends LinearOpMode {

    public Shooter shooter;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // 初始化飞轮系统，根据实际电机名称和反转设置调整
        shooter = new Shooter(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()) {
            // 测试不同速度
            if (gamepad1.aWasPressed()) {
                shooter.setTargetSpeed(850); // 高速
            } else if (gamepad1.bWasPressed()) {
                shooter.setTargetSpeed(760); // 中高速
            } else if (gamepad1.yWasPressed()) {
                shooter.setTargetSpeed(650); // 中速
            } else if (gamepad1.xWasPressed()) {
                shooter.setTargetSpeed(500); // 低速
            } else if (gamepad1.dpad_up) {
                shooter.setTargetSpeed(0); // 停止
            }
            shooter.update();

            telemetry.update();
        }
    }
}