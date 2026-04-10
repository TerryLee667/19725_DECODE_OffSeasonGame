package org.firstinspires.ftc.teamcode.rubbishbin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PIDSVAControllers.PIDController;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import java.util.ArrayList;
import java.util.List;

@Config
public class SpinController {
    //转一圈所需tick
    //todo假如因旋转精度不足导致错位，可以考虑更改unitSpin，或统计旋转ticks数，方便使用后归零处理
    public static double Ticks=10000;
    private int targetTick=0;
    //旋转速度
    public static double SpinPower=0.5;

    public DcMotorEx SpinController;
    public ArtifactDetector sensorDistance1;//吸球口处传感器，1号槽
    public  ArtifactDetector sensorDistance2;//逆时针第二个，2号槽
    public  ArtifactDetector sensorDistance3;//逆时针第三个，3号槽
    //注意：槽位编号为相对于底盘的位置，不随转盘转动而改变
    public  ArtifactDetector sensorDistance4;//测试用，预想为装在炮台下的上升通道内
    //todo届时直接根据电机装配朝向更改sensorDistance1 2 3在上面的顺序，无需变动槽位编号！
    public SpinController spinExecutor;

    Telemetry telemetry;
    //当前速度
    double current_speed = 0;
    //时间变量
    double current_time;
    double previous_time;
    //编码器变量
    double current_encoder = 0;
    //本轮中上次变化前统计的发射球数
    int previous_shooted=0;
    //转盘各槽情况的遍历器
    private int iterator=0;
    //PID切换速度
    public static int PIDSwitchSpeed = 750;
    //PID参数(第一套
    public static double MinPower_1 = 0.1;//最小功率
    public static double k_p_1 = 0.01;
    public static double k_i_1 = 0.2;
    public static double k_d_1 = 0.05;
    public static double max_i_1 = 1;
    //PID参数(第二套
    public static double MinPower_2 = 0.3;
    public static double k_p_2 = 0.011;
    public static double k_d_2 = 0.08;
    public static double k_i_2 = 0.05;
    public static double max_i_2 = 0.6;
    public Rev2mDistanceSensor sensorTimeOfFlight;
    int frameCnt = 0;
    double sumDis = 0;
    double finalDis = 100000;//初始值
    //是否正在旋转
    boolean isSpinning=false;
    //旋转cd（防止错位）
    double spinCd=500;
    //最大发射模式持续时间（Ms）
    double maxShootTime=10000;
    //开始发射时间
    double startShootTime;
    //最大单球发射用时（Ms）
    double maxEachShootTime=3000;
    //单颗球开始发射时间
    double startEachShootTime;
    List<Integer> situation=new ArrayList<>();
    private final PIDController pidController;

    public Telemetry telemetryrc;

    //构造函数//todo检查此处初始化（要加入位置对正，但测试时不必），另外，可能需采用RUN_USING_ENCODER模式编写unitSpin等并加入校准（如果旋转速度太慢的话）
    public SpinController(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName){
            //, boolean ifReverse)
        SpinController = hardwareMap.get(DcMotorEx.class, motorName);
        sensorDistance1=new ArtifactDetector(hardwareMap, telemetryrc,"sensorDistance1");
        sensorDistance2=new ArtifactDetector(hardwareMap, telemetryrc,"sensorDistance2");
        sensorDistance3=new ArtifactDetector(hardwareMap, telemetryrc,"sensorDistance3");
        sensorDistance4=new ArtifactDetector(hardwareMap, telemetryrc,"sensorDistance4");
        SpinController.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SpinController.setTargetPosition(targetTick);
        SpinController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SpinController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        if(ifReverse)
//            SpinController.setDirection(DcMotorSimple.Direction.REVERSE);
//        else
//            SpinController.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetryrc;
        pidController = new PIDController(k_p_1, k_i_1, k_d_1, max_i_1);
        previous_time = System.currentTimeMillis();
    }

    //单位旋转：旋转120°（预想为顺时针）
    public void unitSpin(){
        if(!isSpinning){
            targetTick=targetTick+(int) (Ticks/3);
            SpinController.setTargetPosition(targetTick);
            //SpinController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            isSpinning=!isSpinning;
            previous_time=System.currentTimeMillis();
        }else{
            if (System.currentTimeMillis() - previous_time >= spinCd) {
                isSpinning = !isSpinning;}
        }
    }

    //单位旋转：反向旋转120°
    public void antiUnitSpin(){
        if(!isSpinning){
            targetTick=targetTick-(int) (Ticks/3);
            SpinController.setTargetPosition((int) (-Ticks/3));
            //SpinController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            isSpinning=!isSpinning;
            previous_time=System.currentTimeMillis();
        }else{
            if (System.currentTimeMillis() - previous_time >= spinCd) {
                isSpinning = !isSpinning;}
        }
    }

    //将槽位调整到炮台正下方，再继续进行单位旋转，同时读取持球情况
    public void startShootMode(){
        situation = ArtifactDetector.getSituation(sensorDistance1, sensorDistance2, sensorDistance3);
        targetTick=targetTick+(int) (Ticks/6);
        SpinController.setTargetPosition(targetTick);
        //SpinController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        startShootTime=System.currentTimeMillis();
        startEachShootTime=System.currentTimeMillis();
    }

    //退出发射模式，槽位回到对准吸球口位置，重置射球计数器
    public void quitShootMode(){
        sensorDistance4.counterReset();
        targetTick=targetTick-(int) (Ticks/6);
        SpinController.setTargetPosition(targetTick);
        //SpinController.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //sweeper开启时每帧调用，保证空位永远朝向吸球口
    public void takeSpin() {
        if (sensorDistance1.Whether_get() && !isSpinning) {//这里阻止了isSpinning为真时的执行进入，不能利用单位旋转自带的Cd重置，必须在下方添加重置段
            if (sensorDistance2.Whether_get()) {
                if (!sensorDistance3.Whether_get()) {
                    spinExecutor.antiUnitSpin();
                    //previous_time = System.currentTimeMillis();
                }
            } else {
                spinExecutor.unitSpin();
                //previous_time = System.currentTimeMillis();
            }
        } else {
            if (isSpinning&&System.currentTimeMillis() - previous_time >= spinCd) {
                isSpinning = !isSpinning;
            }
        }
    }

    //炮台开启时每帧调用，保证球位永远朝向发射口
    public void shootSpin() {
        if(sensorDistance4.getShooted()< situation.get(3)&&iterator<3&&System.currentTimeMillis()-startShootTime<=maxShootTime) {//在没射完球、遍历器不触及situation第四项、总时不超时时运行
            if(situation.get(iterator)==1&&System.currentTimeMillis()-startEachShootTime<=maxEachShootTime) {//判断当前遍历的情况元素，若有球且单球不超时时运行
                if ((previous_shooted != sensorDistance4.getShooted()) && !isSpinning) {//监测到有球射出，在可旋转时刻旋转
                    spinExecutor.unitSpin();
                    previous_shooted= sensorDistance4.getShooted();//同步射球数
                }else{
                    if (isSpinning&&System.currentTimeMillis() - previous_time >= spinCd) {//转完一次后进入该检测段，spinCd过去后重置个别检测项，并累加一次遍历器
                        isSpinning = !isSpinning;
                        iterator++;
                        startEachShootTime=System.currentTimeMillis();
                    }
                }
            }else{//若没有球或超时则直接转走
                spinExecutor.unitSpin();
            }
        }else{//射完球或遍历器错误或超时时重置
            iterator=0;
            previous_shooted=0;
        }
    }

    //用于朝向吸球口情况下的校准，在takeSpin结束后调用
    public void takeCalibrate(){
        targetTick=(int)(Ticks*(Math.round(SpinController.getCurrentPosition()/Ticks)));
        SpinController.setTargetPosition(targetTick);
    }

    public boolean Whether_Spinning(){
        return isSpinning;
    }

    public int getPosition(){
        return SpinController.getCurrentPosition();
    }

    public double getPower(){
        return SpinController.getPower();
    }

    public double getVelocity(){
        return SpinController.getVelocity();
    }

    public void setTelemetry(){
        telemetry.addData("Current Position", this.getPosition());
        telemetry.addData("Current Target Position", this.targetTick);
        telemetry.addData("Is Spinning", this.Whether_Spinning());
        telemetry.addData("Spinning Power", this.getPower());
        telemetry.addData("Spinning Velocity", this.getVelocity());
        telemetry.addData("situation",this.situation);
    }
    //没什么用，但保险起见，可以在进入发射模式前检查一下
    public boolean checkSituationList(){
        return situation.size()==4;
    }
}
