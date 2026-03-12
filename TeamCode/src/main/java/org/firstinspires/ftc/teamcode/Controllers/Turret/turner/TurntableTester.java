package org.firstinspires.ftc.teamcode.Controllers.Turret.turner;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "TurntableTester", group = "Tests")
public class TurntableTester extends LinearOpMode{
    public SpinController spinController;
    public Telemetry telemetryrc;
    public boolean takingChanger=false;
    public boolean taking=false;
    public boolean ShootModeChanger=false;
    public boolean inShootMode=false;

    public void runOpMode() throws InterruptedException {
        spinController = new SpinController(hardwareMap, telemetryrc,"turntable");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {//a：顺向单位旋转
                spinController.unitSpin();
            }

            else if (gamepad1.bWasPressed()) {//b：逆向单位旋转
                spinController.antiUnitSpin();
            }

            else if (gamepad1.yWasPressed()&&!takingChanger) {//y：开关takeSpin
                takingChanger=true;
                taking=!taking;
                if(!taking){
                    spinController.takeCalibrate();
                }
            } else if (gamepad1.yWasReleased()&&takingChanger) {
                takingChanger=false;
            }else if(taking){
                spinController.takeSpin();
            }

            else if (gamepad1.xWasPressed()&&!ShootModeChanger) {//x：开关shootSpin
                ShootModeChanger=true;
                inShootMode=!inShootMode;
                if(inShootMode){
                    spinController.startShootMode();
                }else{
                    spinController.quitShootMode();
                }
            } else if (gamepad1.xWasReleased()&&ShootModeChanger) {
                ShootModeChanger=false;
            }else if(inShootMode){
                spinController.shootSpin();


            spinController.setTelemetry();
            telemetry.update();
            }
        }
    }
}
