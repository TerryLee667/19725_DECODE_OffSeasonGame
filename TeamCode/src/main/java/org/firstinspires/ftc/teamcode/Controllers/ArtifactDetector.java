package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@Config
public class ArtifactDetector {
    public  DistanceSensor sensorDistance;

    //todo根据电机装配朝向更改
    public Rev2mDistanceSensor sensorTimeOfFlight;
    Telemetry telemetry;


    public static double maxdistance=20;//此变量代表球吸入时与传感器的距离：最大值
    public static double wrongdistance=30;//那个b传感器，在3cm以内搁那乱转，神经病，写这个变量避免一下这种愚蠢行为。

    public static double maxdistance_shooting=15;//测试用，发射时被发射球与传感器的距离（最大值）

    int frameCnt = 0;
    double sumDis = 0;
    double finalDis = 100000;//初始值
    int shootedCnt=0;//已射出球数计数器
    boolean resetter=false;//用于方便统计当前轮已射出球数


    /**
     *true 表示有3个球
     * false 表示3个球未满（未检测到球）
     */
    public boolean Whether_get(){
        double sensor_distance=sensorDistance.getDistance(DistanceUnit.MM);
        if (sensor_distance <= maxdistance & sensor_distance>=wrongdistance) {
            return true;
        }
        else{
            return false;
        }
    }
    public ArtifactDetector(HardwareMap hardwareMap, Telemetry telemetryrc,String sensorName){
        sensorDistance = hardwareMap.get(DistanceSensor.class, sensorName);
        this.telemetry = telemetryrc;
    }

    public double getDis(DistanceSensor sensorDistance) {
        return sensorDistance.getDistance(DistanceUnit.MM);
    }

    //将某一瞬间的持球情况通过列表方式返回，前三个元素表示各个槽位持球情况（顺序为3号槽、1号槽、2号槽），最后一个表示持球总数（只能在非发射模式下使用）
    public static List<Integer> getSituation(ArtifactDetector disSensor1,ArtifactDetector disSensor2,ArtifactDetector disSensor3){
        int total=0;
        List<Integer> situation=new ArrayList<>();
        if(disSensor3.Whether_get()){
            situation.add(1);
            total++;
        }else{
            situation.add(0);
        }
        if(disSensor1.Whether_get()){
            situation.add(1);
            total++;
        }else{
            situation.add(0);
        }
        if(disSensor2.Whether_get()){
            situation.add(1);
            total++;
        }else{
            situation.add(0);
        }
        situation.add(total);
        return situation;
    }

    //测试用功能，通过距离传感器判断是否有球发射
    public boolean Whether_shoot(){
        double sensor_distance=sensorDistance.getDistance(DistanceUnit.MM);
        if (sensor_distance <= maxdistance_shooting & sensor_distance>=wrongdistance) {
            return true;
        }
        else{
            return false;
        }
    }

    //测试用方法，每帧调用给已射出球计数
    public int getShooted(){
        double sensor_distance=sensorDistance.getDistance(DistanceUnit.MM);
        if(sensor_distance <= maxdistance_shooting & sensor_distance>=wrongdistance&&!resetter){
            shootedCnt++;
            resetter=true;
        }if(!(sensor_distance <= maxdistance_shooting & sensor_distance>=wrongdistance)&&resetter){
            resetter=false;
        }
        return shootedCnt;
    }

    //每次转盘退出发射模式时调用，重置计数器状况
    public void counterReset(){
        resetter=false;
        shootedCnt=0;
    }
}


