package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {
    
    private Turret turret;
    
    @Override
    public void runOpMode() {
        // 初始化 Turret
        turret = new Turret(hardwareMap, telemetry,1.0,0.0,0,"blue",20,24);
        
        turret.set(100, 50); // 设置速度转换参数 k 和 b
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Delta H", turret.getDeltaH());
        telemetry.addData("k", 100);
        telemetry.addData("b", 50);
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // 检查是否按下 A 键
            boolean shouldShoot = gamepad1.a;
            
            // 每帧调用 update 方法
            turret.update(shouldShoot);
            
            // 获取当前角度
            Object[] aimResult = turret.aim();
            boolean isTargetFound = (boolean) aimResult[0];
            double currentRoll = (double) aimResult[1];
            double currentYaw = (double) aimResult[2];
            
            // 显示信息
            telemetry.addData("Status", "Running");
            telemetry.addData("Roll", currentRoll);
            telemetry.addData("Yaw", currentYaw);
            telemetry.addData("Should Shoot", shouldShoot);
            telemetry.update();
        }
        
        // 停止时关闭视觉门户
        turret.close();
    }
}
