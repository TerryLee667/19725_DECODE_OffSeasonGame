package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.WebcamName;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ProcessorTest", group = "Tests")
public class ProcessorTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    private double decimation = 2.0;
    private int whiteBalance = 4000;
    private int exposure = 20;
    private int gain = 50;
    
    private static final double DECIMATION_STEP = 0.1;
    private static final int WHITE_BALANCE_STEP = 100;
    private static final int EXPOSURE_STEP = 5;
    private static final int GAIN_STEP = 5;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        initAprilTag(hardwareMap);
        
        waitForStart();
        
        while (opModeIsActive()) {
            handleGamepadInput();
            
            List<AprilTagDetection> detections = aprilTag.getDetections();
            
            telemetry.addData("=== AprilTag 检测结果 ===", "");
            telemetry.addData("检测到数量", detections.size());
            telemetry.addLine();
            
            for (int i = 0; i < detections.size(); i++) {
                AprilTagDetection detection = detections.get(i);
                telemetry.addData("Tag [" + i + "]", "ID: %d", detection.id);
                telemetry.addData("  水平偏移(bearing)", "%.1f 度", detection.ftcPose.bearing);
                telemetry.addData("  垂直偏移(elevation)", "%.1f 度", detection.ftcPose.elevation);
                telemetry.addData("  距离", "%.1f mm", detection.ftcPose.range);
                telemetry.addData("  俯仰角", "%.1f 度", detection.ftcPose.pitch);
                telemetry.addData("  横滚角", "%.1f 度", detection.ftcPose.roll);
                telemetry.addLine();
            }
            
            telemetry.addData("=== 当前参数 ===", "");
            telemetry.addData("Decimation", "%.1f", decimation);
            telemetry.addData("White Balance", "%d K", whiteBalance);
            telemetry.addData("Exposure", "%d", exposure);
            telemetry.addData("Gain", "%d", gain);
            telemetry.addLine();
            telemetry.addData("=== 控制说明 ===", "");
            telemetry.addData("G1上下", "Decimation +/-%.1f", DECIMATION_STEP);
            telemetry.addData("G1左右", "White Balance +/-%d", WHITE_BALANCE_STEP);
            telemetry.addData("G2上下", "Exposure +/-%d", EXPOSURE_STEP);
            telemetry.addData("G2左右", "Gain +/-%d", GAIN_STEP);
            telemetry.addData("按住RT", "步长减半", "");
            
            telemetry.update();
        }
        
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    private void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDecimation(decimation)
                .build();
        
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    
    private void handleGamepadInput() {
        double stepMultiplier = (gamepad1.right_trigger || gamepad2.right_trigger) > 0.5 ? 0.5 : 1.0;
        
        if (gamepad1.dpad_up) {
            decimation += DECIMATION_STEP * stepMultiplier;
            updateDecimation();
            sleep(100);
        } else if (gamepad1.dpad_down) {
            decimation -= DECIMATION_STEP * stepMultiplier;
            decimation = Math.max(1.0, decimation);
            updateDecimation();
            sleep(100);
        }
        
        if (gamepad1.dpad_left) {
            whiteBalance -= WHITE_BALANCE_STEP * stepMultiplier;
            whiteBalance = Math.max(2000, whiteBalance);
            updateWhiteBalance();
            sleep(100);
        } else if (gamepad1.dpad_right) {
            whiteBalance += WHITE_BALANCE_STEP * stepMultiplier;
            whiteBalance = Math.min(6500, whiteBalance);
            updateWhiteBalance();
            sleep(100);
        }
        
        if (gamepad2.dpad_up) {
            exposure += EXPOSURE_STEP * stepMultiplier;
            exposure = Math.min(204, exposure);
            updateExposure();
            sleep(100);
        } else if (gamepad2.dpad_down) {
            exposure -= EXPOSURE_STEP * stepMultiplier;
            exposure = Math.max(0, exposure);
            updateExposure();
            sleep(100);
        }
        
        if (gamepad2.dpad_left) {
            gain -= GAIN_STEP * stepMultiplier;
            gain = Math.max(0, gain);
            updateGain();
            sleep(100);
        } else if (gamepad2.dpad_right) {
            gain += GAIN_STEP * stepMultiplier;
            gain = Math.min(255, gain);
            updateGain();
            sleep(100);
        }
    }
    
    private void updateDecimation() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDecimation(decimation)
                .build();
        
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    
    private void updateWhiteBalance() {
        if (visionPortal != null) {
            visionPortal.setCameraWhiteBalance(whiteBalance);
        }
    }
    
    private void updateExposure() {
        if (visionPortal != null) {
            visionPortal.setCameraExposure(exposure, gain);
        }
    }
    
    private void updateGain() {
        if (visionPortal != null) {
            visionPortal.setCameraExposure(exposure, gain);
        }
    }
}