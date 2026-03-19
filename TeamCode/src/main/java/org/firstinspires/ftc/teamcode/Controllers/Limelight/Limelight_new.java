package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.nio.channels.ClosedSelectorException;

@Config
@TeleOp(name = "Limelight_new",group = "Tests")
public class Limelight_new extends LinearOpMode {
    public enum ROBOT_STATUS{
        TURNING,
        WAITING,
        STOP;
    }
    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;
    public enum TEAM_COLOR {
        RED,BLUE
    }
    TEAM_COLOR teamColor;
    public enum TRIGGER_STATUS {
        OPEN,
        CLOSE
    }
    TRIGGER_STATUS triggerStatus = TRIGGER_STATUS.CLOSE;
    public enum TURNER_STATUS{
        TURN,
        STOP;
    }
    TURNER_STATUS turnerStatus = TURNER_STATUS.STOP;

    TurnTester_PID turnTesterPid;
    public static boolean isturning = false;
    public static boolean hasTarget = false;


    void Init(){

        //todo set team color
        teamColor = TEAM_COLOR.BLUE;

        turnTesterPid = new TurnTester_PID(hardwareMap);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry = InstanceTelemetry.init(telemetry);

    }
    void turning(){

    }


    @Override
    public void runOpMode() throws InterruptedException {
        //初始化
        Init();
        //
    }

}
