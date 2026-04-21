package org.firstinspires.ftc.teamcode.Library;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * ExampleVelPIDSVAMotor 类是一个示例 TeleOp，展示如何使用 ExampleVoltageOutMotor 控制电机速度
 */
@Config
@TeleOp(name = "Example VoltageOut Motor")
public class ExampleVelPIDSVAMotor extends LinearOpMode {
    /** 目标速度 */
    public static double targetVelocity = 1000;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化遥测，同时输出到Driver Station和Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // 创建ExampleVoltageOutMotor实例，控制名为"SVA"的电机
        ExampleVoltageOutMotor flywheel = new ExampleVoltageOutMotor(hardwareMap, "SVA", telemetry);
        
        waitForStart();
        
        // 主循环
        while (opModeIsActive()) {
            // 用左摇杆Y轴控制目标转速（-1~1 -> -max~max）
            targetVelocity = -gamepad1.left_stick_y * 2000; // 2000为示例最大速度
            
            // 设置目标速度
            flywheel.setTargetVelocity(targetVelocity);
            
            // 更新电机控制
            flywheel.update();
            
            // 按A键停止电机
            if (gamepad1.a) {
                flywheel.setTargetVelocity(0);
            }
            
            // 输出目标速度
            telemetry.addData("Setpoint", targetVelocity);
            telemetry.update();
        }
        
        // 停止电机
        flywheel.stop();
    }
}
