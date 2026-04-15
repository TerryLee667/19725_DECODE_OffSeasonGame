package org.firstinspires.ftc.teamcode.Controllers.Limelight;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Python_Read", group = "Tests")
public class Python_Read extends LinearOpMode {
    private Limelight3A limelight;
    private LLResult result;
    private LLStatus status;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(5); // 切换到 Python 输出管道

        waitForStart();
        while (opModeIsActive()) {

            status = limelight.getStatus();
            result = limelight.getLatestResult();


            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            if (gamepad1.aWasPressed()) {
                limelight.start(); // 开始获取数据

                if (result != null && result.isValid()) {
                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                }
            } else if (gamepad1.bWasPressed()) {
                limelight.stop(); // 停止获取数据
            }

            telemetry.update();
        }
    }
}
