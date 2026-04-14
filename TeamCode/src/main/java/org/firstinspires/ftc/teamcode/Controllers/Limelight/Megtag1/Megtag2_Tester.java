package org.firstinspires.ftc.teamcode.Controllers.Limelight.Megtag1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// IMU 相关 (推荐使用 Universal IMU 接口)
import com.qualcomm.hardware.bosch.BNO055IMU; // 如果你需要使用传统的 BNO055IMU 特定类
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.IMU; // 推荐：使用通用 IMU 接口

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRoll; // 当使用 Universal IMU 接口获取 Yaw/Pitch/Roll 时需要[reference:6]


import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.ImuAngularVelocityResponse;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Controllers.IMUSensor;

@Config
@TeleOp(name = "Megtag2_Tester", group = "Tests")
public class Megtag2_Tester extends LinearOpMode {

    private Limelight3A limelight;
    public LLResult result;
    public LLStatus status;
    Telemetry telemetry;
    IMUSensor imu;
    @Override
    public void runOpMode() throws InterruptedException {

        imu = new IMUSensor(hardwareMap, "imu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)); // 根据你的 IMU 安装方向调整参数

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.pipelineSwitch(0);
        limelight.start();
//        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // 首先，告诉 Limelight 您的机器人面向哪个方向
            double robotYaw = imu.getYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // 获取机器人当前的 Yaw 角度
            limelight.updateRobotOrientation(robotYaw);

             status = limelight.getStatus();
             telemetry.addData("Name", "%s",
                     status.getName());
             telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                     status.getTemp(), status.getCpu(),(int)status.getFps());
             telemetry.addData("Pipeline", "Index: %d, Type: %s",
                     status.getPipelineIndex(), status.getPipelineType());

             result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    sleep(100); // 添加适当的延迟以避免过快更新
                }
            }
        }
    }
}



