package org.firstinspires.ftc.teamcode.Controllers.Limelight;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.Motor_pid;


public class Limelight_Calculater {
    Telemetry telemetry;
    public static double tx = Limelight.tx;
    public static double ty = Limelight.ty;
    public static double txnc = Limelight.txnc;
    public static double tync = Limelight.tync;
    public static boolean hasTarget = false;
    public static boolean isturning = false;
    TurnTester_PID turnTester_pid;


    public void turning(){
        if (!hasTarget){
            isturning = true;
            telemetry.addData("IsTurning","true");

            if (tx > 0) {
                turnTester_pid.targeting(-1);
                telemetry.addData("TurnDirection", "Right");
                telemetry.addData("tx",tx);
            } else if (tx < 0) {
                turnTester_pid.targeting(1);
                telemetry.addData("TurnDirection", "Left");
                telemetry.addData("tx",tx);
            }
            telemetry.addData("tx",tx);
            telemetry.update();
        }else {
            isturning = false;
            turnTester_pid.targeting(0);
            telemetry.addData("IsTurning","false");
            telemetry.addData("tx",tx);
            telemetry.update();
        }

    }

}
