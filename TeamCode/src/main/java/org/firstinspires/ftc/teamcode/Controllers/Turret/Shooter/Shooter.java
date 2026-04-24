package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.SlotConfig;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.VoltageOutMotor;

@Config
public class Shooter {
    public VoltageOutMotor shooterL;
    public VoltageOutMotor shooterR;
    Telemetry telemetry;

    public double targetSpeed = 0;

    public static int PIDSwitchSpeed = 750;

    public SlotConfig slot1 = new SlotConfig()
            .withKP(0.01).withKI(0.05).withKD(0.2).withMaxI(1)
            .withKS(0).withKV(0).withKA(0)
            .withOutputLimits(-14, 14);
    public SlotConfig slot2 = new SlotConfig()
            .withKP(0.011).withKI(0.05).withKD(0.08).withMaxI(0.6)
            .withKS(0).withKV(0).withKA(0)
            .withOutputLimits(-14, 14);

    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc, String nameL,String nameR){
        //TODO:修改reverse
        shooterL= new VoltageOutMotor(hardwareMap,nameL,telemetryrc,false);
        shooterR= new VoltageOutMotor(hardwareMap,nameR,telemetryrc,true);
        this.telemetry=telemetryrc;

    }
    public void setTargetSpeed(int targetSpeed){
        this.targetSpeed=targetSpeed;
        //根据速度选择PID
        if(targetSpeed<PIDSwitchSpeed){
            shooterL.setTargetVelocity(targetSpeed);
            shooterL.setconfig(slot1);
            shooterR.setTargetVelocity(targetSpeed);
            shooterR.setconfig(slot1);
        }
        else {
            shooterL.setTargetVelocity(targetSpeed);
            shooterL.setconfig(slot2);
            shooterR.setTargetVelocity(targetSpeed);
            shooterR.setconfig(slot2);
        }

        if(targetSpeed == 0){
            shooterL.setTargetVelocity(0);
            shooterR.setTargetVelocity(0);

        }

    }
    public boolean whetherReachSpeed(){
   return shooterL.whetherReachTarget()&&shooterR.whetherReachTarget();
    }

    public void update(){
        shooterL.update();
        shooterR.update();
    }
    public double getPowerL(){return shooterL.getPower();}
    public double getPowerR(){return shooterR.getPower();}
    public double getSpeedL(){return shooterL.getVelocity();}
    public double getSpeedR(){return shooterR.getVelocity();}
    public void setTelemetry(){
        telemetry.addData("Shooter PowerL", this.getPowerL());
        telemetry.addData("Shooter PowerR", this.getPowerR());
        telemetry.addData("Shooter Target Speed", this.targetSpeed);
        telemetry.addData("Shooter VelocityL", this.getSpeedL());
        telemetry.addData("Shooter VelocityR", this.getSpeedR());
    }







}
