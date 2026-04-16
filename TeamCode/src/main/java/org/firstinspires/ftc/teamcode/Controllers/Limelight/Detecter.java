//package org.firstinspires.ftc.teamcode.Controllers.Limelight;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import java.util.List;
//import java.util.ArrayList;
//@TeleOp(name = "Limelight Detector Display", group = "Vision")
//public class LimelightDetectorDisplay extends LinearOpMode {
//    private Limelight3A limelight;
//    private LLResult result;
//    private LLStatus status;
//    private Telemetry.Item statusItem;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(5); // �л��� Python ����ܵ�
//        limelight.start();
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//        waitForStart();
//        while (opModeIsActive()) {
//            status = limelight.getStatus();
//            result = limelight.getLatestResult();
//            telemetry.addData("Name", "%s", status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
//            if (result.isValid()) {
//                updateTelemetry(result);
//            } else {
//                telemetry.addData("Limelight", "No data available");
//            }
//            telemetry.update();
//        }
//        limelight.stop();
//    }
//    public double[][] GetCenter(String name, LLResult result) {
//        if (result == null || !result.isValid()) {
//            return new double[0][2];
//        }
//        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//        List<double[]> centers = new ArrayList<>();
//        for (LLResultTypes.DetectorResult dr : detectorResults) {
//            if (dr.getClassName().equals(name)) {
//                double[] bbox = dr.getBoundingBox();
//                if (bbox != null && bbox.length >= 4) {
//                    double centerX = (bbox[0] + bbox[2]) / 2.0;
//                    double centerY = (bbox[1] + bbox[3]) / 2.0;
//                    centers.add(new double[]{centerX, centerY});
//                }
//            }
//        }
//        return centers.toArray(new double[0][2]);
//    }
//    private void updateTelemetry(LLResult result) {
//        telemetry.addData("Timestamp", getRuntime());
//        if (result == null || !result.isValid()) {
//            telemetry.addData("Detection Status", "No valid result");
//            telemetry.update();
//            return;
//        }
//        telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//        List<LLResultTypes.DetectorResult> detectors = result.getDetectorResults();
//        if (detectors == null || detectors.isEmpty()) {
//            telemetry.addData("Objects Found", "0");
//            telemetry.update();
//            return;
//        }
//        telemetry.addData("Objects Found", detectors.size());
//        // �������м�⵽������
//        for (int i = 0; i < detectors.size(); i++) {
//            LLResultTypes.DetectorResult det = detectors.get(i);
//            String className = det.getClassName();
//            double confidence = det.getConfidence();
//            // ��ȡ�߽�򣨹�һ�����꣬[x1, y1, x2, y2]��
//            double[] bbox = det.getBoundingBox();
//            if (bbox == null || bbox.length < 4) {
//                continue;
//            }
//            double x1 = bbox[0];
//            double y1 = bbox[1];
//            double x2 = bbox[2];
//            double y2 = bbox[3];
//            // �������ĵ����꣨��һ������Χ 0~1��
//            double centerX = (x1 + x2) / 2.0;
//            double centerY = (y1 + y2) / 2.0;
//            // ��ѡ��ת��Ϊ�������꣨ͼ��ߴ� 320x320��
//            int pixelX = (int) (centerX * 320);
//            int pixelY = (int) (centerY * 320);
//            // ��ʾ������Ϣ
//            String caption = String.format("Obj_%d", i);
//            String info = String.format("%s (%.2f) | Center: (%.2f, %.2f) [%d, %d]",
//                    className, confidence, centerX, centerY, pixelX, pixelY);
//            telemetry.addData(caption, info);
//        }
//        // Display centers for "target"
//        double[][] centers = GetCenter("target", result);
//        for (int i = 0; i < centers.length; i++) {
//            telemetry.addData("Center " + i, "%.2f, %.2f", centers[i][0], centers[i][1]);
//        }
//        telemetry.update();
//    }
//}
