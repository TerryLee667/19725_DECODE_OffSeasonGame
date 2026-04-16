package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class Detector {
    private Limelight3A limelight;
   // public Telemetry telemetry;

    private static final int PIPELINE_INDEX = 5; // 使用pipeline5
    
    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     */
    public Detector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_INDEX);
//        limelight.start();
    }
    public void start() {
        if (limelight != null) {
            limelight.start();
        }
    }
    
    /**
     * 获取指定对象的中心坐标
     * @param objectName 对象名称
     * @return 二维数组，每行表示一个对象的归一化中心坐标 [x, y]
     */
    public double[][] get_center(String objectName) {
        LLResult result = limelight.getLatestResult();
        List<double[]> centers = new ArrayList<>();


        if (result != null && result.isValid()) {
            // 获取检测器结果
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                // 检查对象名称是否匹配
                if (dr.getClassName().equals(objectName)) {
                    // 获取像素坐标偏移
                    double txp = dr.getTargetXPixels();
                    double typ = dr.getTargetYPixels();
                    
                    // 归一化到0-1范围（假设图像分辨率为640x480）
                    double centerX = (txp + 320) / 640.0;
                    double centerY = (typ + 240) / 480.0;
                    
                    // 确保坐标在0-1范围内
                    centerX = Math.max(0.0, Math.min(1.0, centerX));
                    centerY = Math.max(0.0, Math.min(1.0, centerY));
                    
                    centers.add(new double[]{centerX, centerY});
                }
            }
        }
        
        // 将List转换为二维数组
        double[][] resultArray = new double[centers.size()][2];
        for (int i = 0; i < centers.size(); i++) {
            resultArray[i] = centers.get(i);

            //telemetry.addData("DT", "A1: %s, A2: %.2f", resultArray[i][0], resultArray[i][1]);
           // telemetry.update();
        }
        
        return resultArray;
    }
    
    /**
     * 停止Limelight
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}