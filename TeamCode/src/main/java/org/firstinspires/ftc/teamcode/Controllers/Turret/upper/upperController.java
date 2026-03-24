//package org.firstinspires.ftc.teamcode.Controllers.Turret.upper;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class upperController {
//    double upperspeed=1;
//    boolean ifReverse=true;
//    public boolean isWorking=false;
//    public DcMotorEx motor;
//    public upperController(HardwareMap hardwareMap){
//        this.motor=hardwareMap.get(DcMotorEx.class,"uppermotor");
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    }
//    public boolean ballup(){
//        ifReverse=true;
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(ifReverse){motor.setTargetPosition(1000);}
//        motor.setVelocity(1000);
//        if(motor.getCurrentPosition()==1000&ifReverse){
//            ifReverse=false;
//            motor.setTargetPosition(-1000);
//        }
//        if(!ifReverse&motor.getCurrentPosition()==-1000){
//
//            isWorking=false;
//
//        }
//
//    }
//}
