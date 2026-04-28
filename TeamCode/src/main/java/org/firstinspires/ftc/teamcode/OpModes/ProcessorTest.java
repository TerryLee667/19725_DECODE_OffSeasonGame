package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "AprilTag Tester", group = "Test")
public class ProcessorTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // 调节参数（使用 double 方便步长计算，最终转换为整数）
    private double decimation = 2.0;
    private long exposureMs = 10;        // 曝光时间，单位毫秒
    private double gain = 0;             // 0 通常代表自动，手动时才有意义
    private double whiteBalanceK = 4000; // 色温，单位开尔文

    private ExposureControl exposureControl;
    private GainControl gainControl;
    private WhiteBalanceControl whiteBalanceControl;

    @Override
    public void runOpMode() {
        // 1. 初始化 AprilTag 处理器
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // 2. 初始化 VisionPortal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                    org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                    "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // 3. 等待摄像头就绪
        while (visionPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY) {
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.update();
        }

        // 4. 获取相机控制对象
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        // 5. 设置为手动模式，并读取当前真实值（若支持）
        if (exposureControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureMs = exposureControl.getExposure(TimeUnit.MILLISECONDS);
        }
        if (gainControl != null) {
            gain = gainControl.getGain();
        }
        if (whiteBalanceControl != null) {
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            whiteBalanceK = whiteBalanceControl.getWhiteBalanceTemperature();
        }

        telemetry.addLine("Ready! Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---- 手柄调节逻辑 ----
            boolean fineAdjust = (gamepad1.right_trigger > 0.5) || (gamepad2.right_trigger > 0.5);
            double factor = fineAdjust ? 0.5 : 1.0;

            // 手柄1 上下：调节 Decimation（步长0.1，减半后0.05）
            if (gamepad1.dpad_up) {
                decimation += 0.1 * factor;
            } else if (gamepad1.dpad_down) {
                decimation -= 0.1 * factor;
            }
            decimation = Math.max(1.0, Math.min(8.0, decimation));

            // 手柄1 左右：调节 White Balance（步长100 K，减半后50 K）
            if (gamepad1.dpad_right) {
                whiteBalanceK += 100.0 * factor;
            } else if (gamepad1.dpad_left) {
                whiteBalanceK -= 100.0 * factor;
            }
            // 限制在典型范围内（具体边界可从硬件查询，这里使用常见值）
            whiteBalanceK = Math.max(2000, Math.min(6500, whiteBalanceK));

            // 手柄2 上下：调节 Exposure（步长5 ms，减半后2.5 ms）
            if (gamepad2.dpad_up) {
                exposureMs += 5 * factor;
            } else if (gamepad2.dpad_down) {
                exposureMs -= 5 * factor;
            }
            exposureMs = Math.max(0, Math.min(204, exposureMs)); // 0 ms 表示自动？

            // 手柄2 左右：调节 Gain（步长5，减半后2.5）
            if (gamepad2.dpad_right) {
                gain += 5.0 * factor;
            } else if (gamepad2.dpad_left) {
                gain -= 5.0 * factor;
            }
            gain = Math.max(0, Math.min(255, gain));

            // ---- 将参数应用到硬件 ----
            aprilTag.setDecimation((float) decimation);

            // 曝光与增益：必须在 Manual 模式下设置
            if (exposureControl != null) {
                exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            }
            if (gainControl != null) {
                gainControl.setGain((int) gain);
            }
            if (whiteBalanceControl != null) {
                whiteBalanceControl.setWhiteBalanceTemperature((int) whiteBalanceK);
            }

            // ---- 获取检测结果并显示 ----
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addLine("==== Parameters ====");
            telemetry.addData("Decimation", "%.2f", decimation);
            telemetry.addData("Exposure (ms)", "%d", exposureMs);
            telemetry.addData("Gain", "%.0f", gain);
            telemetry.addData("White Balance (K)", "%.0f", whiteBalanceK);
            telemetry.addData("Fine adjust", fineAdjust);

            telemetry.addLine("\n==== Detections ====");
            telemetry.addData("Count", detections.size());

            if (detections.isEmpty()) {
                telemetry.addLine("No tags detected");
            } else {
                for (AprilTagDetection det : detections) {
                    telemetry.addLine();
                    telemetry.addData("  ID", det.id);
                    if (det.ftcPose != null) {
                        telemetry.addData("    Bearing (deg)", "%.1f", det.ftcPose.bearing);
                        telemetry.addData("    Elevation (deg)", "%.1f", det.ftcPose.elevation);
                        telemetry.addData("    Range (inches)", "%.1f", det.ftcPose.range);
                    } else {
                        telemetry.addLine("    Pose unavailable");
                    }
                }
            }
            telemetry.update();
        }

        visionPortal.close();
    }
}