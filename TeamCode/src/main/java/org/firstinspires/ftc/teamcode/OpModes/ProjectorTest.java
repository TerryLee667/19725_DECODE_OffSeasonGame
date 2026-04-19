package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Limelight.Detector;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.projection.Projector;

@Config
@TeleOp(name = "ProjectorTest", group = "Tests")
public class ProjectorTest extends LinearOpMode {

    private Detector detector;
    private Projector projector;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        detector = new Detector(hardwareMap);
        projector = new Projector(0.9, 0.1);

        detector.start();

        telemetry.addLine("Projector Test Started");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("=== Purple Balls ===");
            double[][] purpleDetections = detector.get_center("purple");
            if (purpleDetections.length == 0) {
                telemetry.addLine("No purple balls detected");
            } else {
                for (int i = 0; i < purpleDetections.length; i++) {
                    double m = purpleDetections[i][0];
                    double n = purpleDetections[i][1];
                    double[] worldPos = projector.project(m, n);
                    double deltaX = worldPos[0];
                    double deltaY = worldPos[1];
                    telemetry.addData("Purple[" + i + "] m", m);
                    telemetry.addData("Purple[" + i + "] n", n);
                    telemetry.addData("Purple[" + i + "] deltaX", deltaX);
                    telemetry.addData("Purple[" + i + "] deltaY", deltaY);
                }
            }

            telemetry.addLine("=== Green Balls ===");
            double[][] greenDetections = detector.get_center("green");
            if (greenDetections.length == 0) {
                telemetry.addLine("No green balls detected");
            } else {
                for (int i = 0; i < greenDetections.length; i++) {
                    double m = greenDetections[i][0];
                    double n = greenDetections[i][1];
                    double[] worldPos = projector.project(m, n);
                    double deltaX = worldPos[0];
                    double deltaY = worldPos[1];
                    telemetry.addData("Green[" + i + "] m", m);
                    telemetry.addData("Green[" + i + "] n", n);
                    telemetry.addData("Green[" + i + "] deltaX", deltaX);
                    telemetry.addData("Green[" + i + "] deltaY", deltaY);
                }
            }

            telemetry.addData("Purple Count", purpleDetections.length);
            telemetry.addData("Green Count", greenDetections.length);

            telemetry.update();
        }

        detector.stop();
    }
}