//package org.firstinspires.ftc.teamcode.Controllers.Limelight;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.List;
//
//@TeleOp(name = "Limelight Detector Display", group = "Vision")
//public class LimelightDetectorDisplay extends LinearOpMode {
//
//    private Limelight3A limelight;
//    private Telemetry.Item statusItem;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // 获取 Limelight 硬件对象
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        // 设置检测结果更新频率（100Hz）
//        limelight.setPollRateHz(100);
//        // 启动视觉处理
//        limelight.start();
//
//        // 切换到神经网络检测 pipeline（索引 0，请根据你的配置调整）
//        limelight.pipelineSwitch(0);
//
//        // 等待开始
//        telemetry.addData("Status", "Initialized. Waiting for start...");
//        telemetry.update();
//        waitForStart();
//
//        // 用于控制显示频率的计时器
//        double lastDisplayTime = 0;
//        final double DISPLAY_INTERVAL_SEC = 1.0; // 每秒更新一次
//
//        while (opModeIsActive()) {
//            // 获取最新的检测结果
//            LLResult result = limelight.getLatestResult();
//
//            // 只每秒更新一次 Telemetry
//            double currentTime = getRuntime();
//            if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL_SEC) {
//                lastDisplayTime = currentTime;
//                updateTelemetry(result);
//            }
//
//            // 可在此处添加机器人的其他控制逻辑
//            // 如果不需其他操作，可以短暂休眠以降低 CPU 占用
//            sleep(20);
//        }
//
//        // 停止摄像头
//        limelight.stop();
//    }
//
//    private void updateTelemetry(LLResult result) {
//        telemetry.addData("Timestamp", getRuntime());
//
//        if (result == null || !result.isValid()) {
//            telemetry.addData("Detection Status", "No valid result");
//            telemetry.update();
//            return;
//        }
//
//        List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();
//        if (detectors == null || detectors.isEmpty()) {
//            telemetry.addData("Objects Found", "0");
//            telemetry.update();
//            return;
//        }
//
//        telemetry.addData("Objects Found", detectors.size());
//
//        // 遍历所有检测到的物体
//        for (int i = 0; i < detectors.size(); i++) {
//            LLResultTypes.DetectorResult det = detectors.get(i);
//            String className = det.getClassName();
//            double confidence = det.getConfidence();
//
//            // 获取边界框（归一化坐标，[x1, y1, x2, y2]）
//            double[] bbox = det.getBoundingBox();
//            if (bbox == null || bbox.length < 4) {
//                continue;
//            }
//            double x1 = bbox[0];
//            double y1 = bbox[1];
//            double x2 = bbox[2];
//            double y2 = bbox[3];
//
//            // 计算中心点坐标（归一化，范围 0~1）
//            double centerX = (x1 + x2) / 2.0;
//            double centerY = (y1 + y2) / 2.0;
//
//            // 可选：转换为像素坐标（图像尺寸 320x320）
//            int pixelX = (int) (centerX * 320);
//            int pixelY = (int) (centerY * 320);
//
//            // 显示物体信息
//            String caption = String.format("Obj_%d", i);
//            String info = String.format("%s (%.2f) | Center: (%.2f, %.2f) [%d, %d]",
//                    className, confidence, centerX, centerY, pixelX, pixelY);
//            telemetry.addData(caption, info);
//        }
//
//        telemetry.update();
//    }
//}