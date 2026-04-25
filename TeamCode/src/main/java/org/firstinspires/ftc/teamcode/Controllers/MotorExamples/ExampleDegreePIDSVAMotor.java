package org.firstinspires.ftc.teamcode.Controllers.MotorExamples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "ExampleDegreePIDSVAMotor", group = "Examples")
public class ExampleDegreePIDSVAMotor extends LinearOpMode {
    public static double DeltaAngle = 1;
    public  static double TargetAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // 初始化控角度电机
        VoltageOutMotor_Degree degreeMotor = new VoltageOutMotor_Degree(
                hardwareMap,
                "motor",
                telemetry,
                false // 是否反转
        );


        double AdditionDegree = 0;
        degreeMotor.setTargetDegree(TargetAngle);

        telemetry.addLine("Ready. Use gamepad to change target angle.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                AdditionDegree += DeltaAngle;
            } else if (gamepad1.dpad_down) {
                AdditionDegree -= DeltaAngle;
            }
            degreeMotor.setTargetDegree(TargetAngle + AdditionDegree);

            // 更新电机控制
            degreeMotor.update();

            // 输出当前状态
            telemetry.addData("additionDegree", AdditionDegree);
            telemetry.addData("TargetAngle", TargetAngle);
            telemetry.addData("CurrentDegree", degreeMotor.getDegree());
            telemetry.addData("Power", degreeMotor.getPower());
            telemetry.addData("ReachedTarget", degreeMotor.ReachedTarget());
            telemetry.update();

            sleep(20); // 控制循环频率
        }

        // 停止电机
        degreeMotor.stop();
    }
}