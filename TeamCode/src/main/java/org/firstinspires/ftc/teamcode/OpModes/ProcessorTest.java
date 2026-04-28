package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Tester", group = "Test")
public class ProcessorTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // 调节参数（使用double以支持减半步长）
    private double decimation = 2.0;
    private double exposure = 10;
    private double gain = 0;          // 0 通常代表自动
    private double whiteBalance = 4000;

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
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
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

        // 5. 读取当前相机参数作为初始值（如果有）
        if (exposureControl != null) {
            exposure = exposureControl.getExposure();
        }
        if (gainControl != null) {
            gain = gainControl.getGain();
        }
        if (whiteBalanceControl != null) {
            whiteBalance = whiteBalanceControl.getWhiteBalance();
        }

        telemetry.addLine("Ready! Press START");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---- 手柄调节逻辑 ----
            // 右扳机（至少一个按住）则步长减半
            boolean fineAdjust = (gamepad1.right_trigger > 0.5) || (gamepad2.right_trigger > 0.5);
            double factor = fineAdjust ? 0.5 : 1.0;

            // 手柄1 上下键调节 Decimation（步长0.1，减半后0.05）
            if (gamepad1.dpad_up) {
                decimation += 0.1 * factor;
            } else if (gamepad1.dpad_down) {
                decimation -= 0.1 * factor;
            }
            decimation = Math.max(1.0, Math.min(8.0, decimation)); // 限制范围

            // 手柄1 左右键调节 White Balance（步长100，减半后50）
            if (gamepad1.dpad_right) {
                whiteBalance += 100.0 * factor;
            } else if (gamepad1.dpad_left) {
                whiteBalance -= 100.0 * factor;
            }
            whiteBalance = Math.max(2000, Math.min(6500, whiteBalance));

            // 手柄2 上下键调节 Exposure（步长5 ms，减半后2.5 ms）
            if (gamepad2.dpad_up) {
                exposure += 5.0 * factor;
            } else if (gamepad2.dpad_down) {
                exposure -= 5.0 * factor;
            }
            exposure = Math.max(0, Math.min(204, exposure));

            // 手柄2 左右键调节 Gain（步长5，减半后2.5）
            if (gamepad2.dpad_right) {
                gain += 5.0 * factor;
            } else if (gamepad2.dpad_left) {
                gain -= 5.0 * factor;
            }
            gain = Math.max(0, Math.min(255, gain));

            // ---- 将参数应用到硬件 ----
            aprilTag.setDecimation((float) decimation);

            // 设置相机参数（需要转换为整数）
            if (exposureControl != null) {
                exposureControl.setExposure((int) exposure);
            }
            if (whiteBalanceControl != null) {
                whiteBalanceControl.setWhiteBalance((int) whiteBalance);
            }
            if (gainControl != null) {
                gainControl.setGain((int) gain);
            }

            // ---- 获取检测结果并显示 ----
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addLine("==== Parameters ====");
            telemetry.addData("Decimation", "%.2f", decimation);
            telemetry.addData("Exposure (ms)", "%.1f (%d)", exposure, (int) exposure);
            telemetry.addData("Gain", "%.1f (%d)", gain, (int) gain);
            telemetry.addData("White Balance (K)", "%.1f (%d)", whiteBalance, (int) whiteBalance);
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

        // 关闭 VisionPortal
        visionPortal.close();
    }
}