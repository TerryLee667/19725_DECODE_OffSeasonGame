package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Shootertester", group = "Tests")
public class Shootertester extends LinearOpMode {

    public ShooterAction shooterAction;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // 初始化双飞轮系统，根据实际电机名称和反转设置调整
        shooterAction = new ShooterAction(hardwareMap, telemetry, "left_shooter", "right_shooter", true, false);
        waitForStart();
        while (opModeIsActive()) {
            // 测试不同速度
            if (gamepad1.aWasPressed()) {
                shooterAction.shoot(850); // 高速
            } else if (gamepad1.bWasPressed()) {
                shooterAction.shoot(760); // 中高速
            } else if (gamepad1.yWasPressed()) {
                shooterAction.shoot(650); // 中速
            } else if (gamepad1.xWasPressed()) {
                shooterAction.shoot(500); // 低速
            } else if (gamepad1.dpad_up) {
                shooterAction.shoot(0); // 停止
            } else if (gamepad1.dpad_down) {
                shooterAction.block(); // 阻塞
            }
            
            // 遥测数据
            shooterAction.setTelemetry();
            telemetry.update();
        }
    }
}