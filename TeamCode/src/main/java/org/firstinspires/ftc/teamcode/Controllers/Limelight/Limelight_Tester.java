package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import static org.firstinspires.ftc.teamcode.Controllers.Limelight.Limelight_Calculater.tx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Limelight_Calculater;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.TurnTester_PID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Limelight_Tester", group = "Tests")
@Config
public class Limelight_Tester extends LinearOpMode {
    Limelight_Calculater limelight_calculater;
    TurnTester_PID turnTesterPid;
    private Limelight3A limelight;
    public static double tx;
    public static double ty;
    public static double txnc;
    public static double tync;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.yWasPressed()) {
                turnTesterPid = new TurnTester_PID(hardwareMap);
                limelight_calculater.turning(true);
                LLStatus status = limelight.getStatus();
                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(),(int)status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());
                if (limelight.getLatestResult() != null) {
                    LLResult result = limelight.getLatestResult();

                    if (result.isValid()) {
                        // Access general information
                        Pose3D botpose = result.getBotpose();
                        double captureLatency = result.getCaptureLatency();
                        double targetingLatency = result.getTargetingLatency();
                        double parseLatency = result.getParseLatency();
                        tx = result.getTx();
                        ty = result.getTy();
                        txnc = result.getTxNC();
                        tync = result.getTyNC();
                        telemetry.addData("tx", tx);
                        telemetry.addData("txnc", txnc);
                        telemetry.addData("ty", ty);
                        telemetry.addData("tync", tync);

                        telemetry.addData("Botpose", botpose.toString());
                    }
                }
            }
            limelight.stop();
        }
    }


}
