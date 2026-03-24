package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.channels.ClosedSelectorException;

@Config
@TeleOp(name = "Limelight_new",group = "Tests")
public class Limelight_new extends LinearOpMode {
    private Limelight3A limelight;
    public Telemetry telemetry;
    public double TargetXDegreesmax = 10;
    public double TargetXDegreesmin = -10;
    public double TargetSpeed = 30;
    public double StopSpeed = 0;
    public double TargetAngle = 0;
    Motor_PID Motor_PID = new Motor_PID();
    LLStatus status = limelight.getStatus();
    LLResult result = limelight.getLatestResult();
    public double tx;
    public double ty;
    public double txnc;
    public double tync;
    public double maxtx = 1;
    public double mintx = -1;

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

    public enum TRIGGER_STATUS {
        OPEN,
        CLOSE
    }

    TRIGGER_STATUS triggerStatus = TRIGGER_STATUS.CLOSE;

    public enum TURNER_STATUS {
        TURN,
        STOP;
    }

    TURNER_STATUS turnerStatus = TURNER_STATUS.STOP;

    TurnTester_PID turnTesterPid;
    public  boolean isturning = false;
    public  boolean hasTarget = false;


    void Init() {

        //todo set team color
        teamColor = TEAM_COLOR.BLUE;



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry = InstanceTelemetry.init(telemetry);

    }
    void turning(double maxtx, double mintx) {
        if (result.isValid()){
            if (result.getTx() > maxtx || result.getTx() < mintx ){
                hasTarget = false;
            }
            if (!hasTarget) {
                isturning = true;
                telemetry.addData("IsTurning", "true");
    //            if (tx > maxtx) {
    //                Motor_PID.turn(-1,);
    //                telemetry.addData("TurnDirection", "Right");
    //                telemetry.addData("tx", tx);
    //            } else if (tx < 0) {
    //                turnTesterPid.targeting(1);
    //                telemetry.addData("TurnDirection", "Left");
    //                telemetry.addData("tx", tx);
    //            }
    //            telemetry.addData("tx", tx);
    //            telemetry.update();
    //        }else {
    //            isturning = false;
    //            hasTarget = true;
    //            turnTesterPid.targeting(0);
    //            telemetry.addData("IsTurning", "false");
    //            telemetry.addData("tx", tx);
    //            telemetry.update();
    //        }
                if (result.getTx() > TargetXDegreesmax) {
                    Motor_PID.turn(TargetSpeed, TargetAngle, result.getTx());
                    telemetry.addData("TurnAction: ","Left_Turning");
                    telemetry.addData("TurnSpeed: ",Motor_PID.getVelocity());
                    hasTarget = false;
                }
                else if (result.getTx() < TargetXDegreesmin){

                    Motor_PID.turn(-TargetSpeed, TargetAngle, result.getTx());
                    telemetry.addData("TurnAction: ","Right_Turning");
                    telemetry.addData("TurnSpeed: ",Motor_PID.getVelocity());
                    hasTarget = false;
                }
                else {
                    Motor_PID.block();
                    telemetry.addData("TurnAction: ","Stop");
                    isturning = false;
                    hasTarget = true;
                }
            }

        }
    }



    @Override
    public void runOpMode() throws InterruptedException {
        //初始化
        Init();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */

        telemetry.update();



        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.yWasPressed()){
                limelight.start();
                telemetry.update();
                turning(maxtx,mintx);
            }
        }


        //
    }

}
