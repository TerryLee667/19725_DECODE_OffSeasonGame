package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class RobotPosition {

    static MecanumDrive drive;
    HardwareMap hardwareMap;
    Localizer localizer;

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    private static RobotPosition instance;

    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }
    private RobotPosition(){
    }

;
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose) {

        instance=new RobotPosition();
        instance.hardwareMap = hardwareMap;

        instance.currentPose = initpose != null ? initpose : new Pose2d(0,0,0);
        instance.drive=new MecanumDrive(hardwareMap,instance.currentPose);
        instance.localizer=instance.drive.localizer;
        return instance;
    }

    // 每帧调用：更新定位器并返回当前位姿
    public Pose2d update() {
        currentVelocity2d = drive.updatePoseEstimate();


        if (instance.localizer != null) {
            try {
                instance.localizer.update();
                Pose2d p = instance.localizer.getPose();
                if (p != null) {
                    instance.currentPose = p;
                }
            } catch (Exception ignored) {
                // 如果 localizer 的方法抛异常，保持现有 pose
            }
        }
        return instance.currentPose;
    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }
    public double getVx(){return  currentVelocity2d.linearVel.x;}
    public double getVy(){return  currentVelocity2d.linearVel.y;}
    public double getOmega(){return currentVelocity2d.angVel;}


}
