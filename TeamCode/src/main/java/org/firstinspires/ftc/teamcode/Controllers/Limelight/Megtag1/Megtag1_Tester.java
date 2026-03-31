package org.firstinspires.ftc.teamcode.Controllers.Limelight.Megtag1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Limelight_new;

@Config
@TeleOp(name = "Megtag1_Tester", group = "Tests")
public class Megtag1_Tester extends LinearOpMode {
    private Limelight3A limelight;
    private Telemetry telemetry;
    LLResult result = limelight.getLatestResult();
    LLStatus status = limelight.getStatus();
    public enum ROBOT_STATUS {
        TURNING,
        WAITING,
        STOP;
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;

    public enum TEAM_COLOR {
        RED, BLUE
    }

    TEAM_COLOR teamColor;
    void Init() {

        //todo set team color
        teamColor = TEAM_COLOR.BLUE;



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry = InstanceTelemetry.init(telemetry);

    }
    @Override
    public void runOpMode() throws InterruptedException {
        //初始化
        Init();
        waitForStart();
        while (opModeIsActive()) {
        }

    }
}
