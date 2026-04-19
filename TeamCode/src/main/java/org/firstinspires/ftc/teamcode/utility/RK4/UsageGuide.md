# RK4 跑打算法使用指南

本文档详细说明如何使用 RK4 跑打算法代码测定参数和预测炮台角度，以实现精确的目标命中。

## 目录

- [1. 系统架构](#1-系统架构)
- [1.1 类依赖关系](#11-类依赖关系)
- [2. 参数标定流程](#2-参数标定流程)
  - [2.1 快速标定（推荐）](#21-快速标定推荐)
  - [2.2 手动标定](#22-手动标定)
  - [2.3 初速度 v0 测定](#23-初速度-v0-测定)
  - [2.4 阻力参数 k 和 n 测定](#24-阻力参数-k-和-n-测定)
- [3. 预测角度使用方法](#3-预测角度使用方法)
  - [3.0 高度差处理说明](#30-高度差处理说明)
  - [3.1 基本使用](#31-基本使用)
  - [3.2 实时控制](#32-实时控制)
  - [3.3 性能优化](#33-性能优化)
- [4. 常见问题与解决方案](#4-常见问题与解决方案)
- [5. 代码示例](#5-代码示例)

## 1. 系统架构

RK4 跑打算法由以下核心组件组成：

| 组件 | 功能 | 文件 | 测定后删除 |
|------|------|------|-----------|
| **ProjectileState** | 小球状态管理 | `ProjectileState.java` | 否 |
| **ProjectileParameters** | 物理参数管理 | `ProjectileParameters.java` | 否 |
| **ProjectileDynamics** | 运动微分方程 | `ProjectileDynamics.java` | 否 |
| **TrajectorySimulator** | 轨迹仿真（RK4 积分） | `TrajectorySimulator.java` | 否 |
| **RobotState** | 机器人状态管理 | `RobotState.java` | 否 |
| **TargetPredictor** | 目标位置预测 | `TargetPredictor.java` | 否 |
| **Solver** | 核心求解器 | `Solver.java` | 否 |
| **CalibrationHelper** | 参数标定辅助 | `CalibrationHelper.java` | **是** |
| **ParameterCalculator** | CSV 参数计算 | `ParameterCalculator.java` | **是** |
| **ParameterCalibrationApp** | 标定应用程序 | `ParameterCalibrationApp.java` | **是** |

### 测定后需删除的文件

以下文件仅用于参数测定，测定完成后可删除：

- `CalibrationHelper.java`
- `ParameterCalculator.java`
- `ParameterCalibrationApp.java`
- `velocity_calibration.csv`
- `drag_calibration.csv`

## 1.1 类依赖关系

```
┌─────────────────────────────────────────────────────────────┐
│                    参数标定工具（测定后删除）                    │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────────┐      ┌─────────────────────────┐   │
│  │ CalibrationHelper    │──────│ TrajectorySimulator    │   │
│  │ [参数标定核心算法]     │      │ [轨迹仿真器]              │   │
│  └─────────────────────┘      └───────────┬─────────────┘   │
│                                          │                  │
│  ┌─────────────────────┐                  │                  │
│  │ ParameterCalculator │──────┐           │                  │
│  │ [CSV数据计算]        │      │           │                  │
│  └─────────────────────┘      │           │                  │
│                                │           │                  │
│  ┌─────────────────────┐      └───────────┤                  │
│  │ ParameterCalibrationApp│───────────────┤                  │
│  │ [命令行应用程序]      │                  │                  │
│  └─────────────────────┘                  │                  │
└──────────────────────────────────────────┴──────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    核心计算模块（保留）                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────┐      ┌─────────────────────────────┐    │
│  │ ProjectileState │◄─────│ TrajectorySimulator          │    │
│  │ [小球状态]       │      │ [RK4积分仿真]                 │    │
│  └─────────────────┘      └──────────────┬──────────────┘    │
│                                          │                    │
│  ┌─────────────────┐                      │                    │
│  │ ProjectileParameters│                   │                    │
│  │ [物理参数]       │◄──────────────────────┘                    │
│  └─────────────────┘                                          │
│                                                              │
│  ┌─────────────────┐      ┌─────────────────────────────┐    │
│  │ RobotState      │──────│ Solver                       │    │
│  │ [机器人状态]     │      │ [双层迭代求解器]              │    │
│  └─────────────────┘      └──────────────┬──────────────┘    │
│                                          │                    │
│  ┌─────────────────┐                      │                    │
│  │ TargetPredictor  │──────────────────────┘                    │
│  │ [目标预测器]     │                                             │
│  └─────────────────┘                                             │
│                                                              │
│  ┌─────────────────┐                                           │
│  │ ProjectileDynamics│ [运动微分方程]                           │
│  │ [RK4计算核心]    │                                           │
│  └─────────────────┘                                           │
└──────────────────────────────────────────────────────────────┘
```

### 核心类说明

| 类 | 依赖 | 说明 |
|---|------|------|
| **ProjectileState** | 无 | 小球运动状态（位置、速度、时间） |
| **ProjectileParameters** | 无 | 物理参数（v₀, k, n, m, g, Δh） |
| **ProjectileDynamics** | ProjectileState, ProjectileParameters | 运动微分方程，计算加速度 |
| **TrajectorySimulator** | ProjectileState, ProjectileParameters, ProjectileDynamics | RK4 数值积分，仿真轨迹 |
| **RobotState** | 无 | 机器人状态（位置、速度） |
| **TargetPredictor** | 无 | 匀速直线运动模型，预测目标位置 |
| **Solver** | TrajectorySimulator, RobotState, TargetPredictor, ProjectileParameters | 双层迭代求解炮台角度 |
| **CalibrationHelper** | TrajectorySimulator | 参数标定核心算法 |
| **ParameterCalculator** | CalibrationHelper | CSV 文件读取和结果封装 |
| **ParameterCalibrationApp** | ParameterCalculator | 命令行交互界面 |

### 依赖关系详解

1. **Solver** 是核心入口，依赖以下类：
   - `TrajectorySimulator`：进行轨迹仿真
   - `RobotState`：获取机器人状态
   - `TargetPredictor`：预测目标位置
   - `ProjectileParameters`：获取物理参数

2. **TrajectorySimulator** 依赖：
   - `ProjectileState`：表示小球状态
   - `ProjectileParameters`：获取物理参数
   - `ProjectileDynamics`：计算加速度

3. **CalibrationHelper** 依赖：
   - `TrajectorySimulator`：用于参数拟合时的轨迹仿真

4. **ParameterCalculator** 依赖：
   - `CalibrationHelper`：调用标定算法

5. **ParameterCalibrationApp** 依赖：
   - `ParameterCalculator`：获取标定结果

### 最小依赖集

比赛代码中只需保留以下核心类即可进行角度计算：

- `ProjectileState.java`
- `ProjectileParameters.java`
- `ProjectileDynamics.java`
- `TrajectorySimulator.java`
- `RobotState.java`
- `TargetPredictor.java`
- `Solver.java`

## 2. 参数标定流程

### 2.1 快速标定（推荐）

使用 `ParameterCalibrationApp.java` 应用程序，可以从 CSV 文件自动读取实验数据并计算所有参数。

#### 准备工作

1. **采集实验数据**：
   - 水平平射数据：发射小球，测量落地点到炮口的水平距离，重复 5-10 次
   - 阻力参数数据：在不同距离设置靶标，调整仰角直到命中，记录 `(距离, 仰角)` 数据对

2. **准备 CSV 文件**：
   - `velocity_calibration.csv`：每行一个水平距离数据
   - `drag_calibration.csv`：每行两个数据点，格式为 `距离, 仰角（度）`

#### 运行标定程序

```bash
# 编译 Java 代码
javac -encoding UTF-8 -d ./out ./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/utility/RK4/*.java

# 运行标定程序
java -cp "./out;./TeamCode/src/main/java" org.firstinspires.ftc.teamcode.utility.RK4.ParameterCalibrationApp
```

#### 程序界面

程序会提示输入：
- RK4 目录路径（包含 CSV 文件的目录）
- 炮口高度（米）
- 小球质量（公斤）

#### 输出结果

程序会输出：
```
=====================================
===== 标定结果 =====
初速度 (v0): 6.250 m/s
阻力系数 (k): 0.012500
速度指数 (n): 1.950
拟合总误差: 0.001234
均方根误差: 0.015811
初速度数据点: 5
阻力参数数据点: 5
状态: 成功

===== 推荐代码 =====
params.v0 = 6.250;
params.k = 0.012500;
params.n = 1.950;
=====================================
```

### 2.2 手动标定

如果无法使用应用程序，可以手动进行参数标定。

**目的**：测定小球离开炮口时的初速度。

**步骤**：

1. **准备工作**：
   - 将机器人静置在平整地面
   - 炮塔水平固定（仰角 θ = 0°）
   - 测量炮口离地高度 H（例如 0.2m）

2. **数据采集**：
   - 发射小球，测量首次落地点到炮口垂足的水平距离 R
   - 重复 5-10 次，计算平均值 `averageRange`

3. **代码实现**：

```java
// 创建标定辅助类
CalibrationHelper calibrator = new CalibrationHelper();

// 创建默认参数
ProjectileParameters params = new ProjectileParameters();
params.deltaH = 0.2; // 炮口高度

// 标定初速度
double measuredRange = 2.5; // 实际测量的水平距离（米）
double v0 = calibrator.calibrateV0(measuredRange, params.deltaH, params);

// 输出结果
System.out.println("标定得到的初速度: " + v0 + " m/s");
```

### 2.4 阻力参数 k 和 n 测定

**目的**：通过不同距离的射击数据拟合空气阻力模型。

**步骤**：

1. **数据采集**：
   - 在 1.5m、2.0m、2.5m、3.0m、3.5m 处设置靶标
   - 对每个距离，调整炮塔仰角直到连续命中
   - 记录数据对 `(距离, 仰角)`

2. **代码实现**：

```java
// 准备采集的数据（距离米, 仰角弧度）
double[][] rangeAngleData = {
    {1.5, Math.toRadians(10)},
    {2.0, Math.toRadians(15)},
    {2.5, Math.toRadians(20)},
    {3.0, Math.toRadians(25)},
    {3.5, Math.toRadians(30)}
};

// 已知初速度
ProjectileParameters params = new ProjectileParameters();
params.v0 = 6.0; // 已标定的初速度

// 拟合阻力参数
CalibrationHelper calibrator = new CalibrationHelper();
CalibrationHelper.CalibrationData result = calibrator.fitDragParameters(rangeAngleData, params);

// 输出结果
System.out.println("拟合得到的阻力参数:");
System.out.println("k: " + result.k);
System.out.println("n: " + result.n);
System.out.println("拟合误差: " + result.totalError);
```

## 3. 预测角度使用方法

### 3.0 高度差处理说明

**重要说明**：本实现只考虑炮口与目标的高度差（`deltaH`），不考虑绝对高度。

- **高度差定义**：`deltaH = 目标高度 - 炮口高度`（单位：米）
- **计算逻辑**：
  1. 所有 z 坐标计算都从 0 开始（炮口高度作为参考点）
  2. 目标高度为 `deltaH`
  3. 轨迹计算只关注相对高度变化

**优势**：
- 简化计算，减少参数传递
- 不依赖于绝对坐标系的建立
- 更符合实际比赛中的使用场景

**使用建议**：
- 测量并设置准确的 `deltaH` 值
- 确保炮台高度在比赛中保持一致



### 3.1 基本使用

**功能**：给定目标位置和机器人状态，计算炮塔朝向和仰角。

#### 方法一：完整参数输入

```java
// 1. 创建求解器
Solver solver = new Solver();

// 2. 设置物理参数（使用标定结果）
ProjectileParameters params = new ProjectileParameters();
params.v0 = 6.0;    // 已标定的初速度
params.k = 0.015;   // 已标定的阻力系数
params.n = 2.0;     // 已标定的速度指数
params.m = 0.1;     // 小球质量（公斤）
params.deltaH = 0.5; // 炮口与目标高度差（米）- 只考虑高度差，不考虑绝对高度
params.thetaMax = Math.PI / 4; // 最大仰角（45度）
solver.setParameters(params);

// 3. 准备输入数据
RobotState robot = new RobotState(0, 0, 0.5, 0.3); // (x,y,vx,vy)
TargetPredictor.TargetState target = new TargetPredictor.TargetState(3, 2, 0, 0.5); // (x,y,vx,vy)

// 4. 求解
Solver.SolverResult result = solver.solve(target.x, target.y, robot, target);

// 5. 处理结果
if (result.success) {
    double turretAngle = result.getTurretPhiDegrees(); // 炮塔水平朝向（度）
    double elevationAngle = result.getThetaDegrees();   // 炮管仰角（度）
    double flightTime = result.flightTime;             // 飞行时间（秒）
    
    System.out.println("炮塔朝向: " + turretAngle + " 度");
    System.out.println("炮管仰角: " + elevationAngle + " 度");
    System.out.println("飞行时间: " + flightTime + " 秒");
} else {
    System.out.println("求解失败: " + result.message);
}
```

#### 方法二：简化参数输入（推荐）

**适用场景**：目标相对地面静止，高于小车且高度差已知。

**参数说明**：
- `relativeX`：目标相对于小车的 x 坐标（小车正前方为 x 正方向）
- `relativeY`：目标相对于小车的 y 坐标
- `robotVx`：小车在 x 方向的速度
- `robotVy`：小车在 y 方向的速度

**代码实现**：

```java
// 1. 创建求解器
Solver solver = new Solver();

// 2. 设置物理参数（使用标定结果）
ProjectileParameters params = new ProjectileParameters();
params.v0 = 6.0;    // 已标定的初速度
params.k = 0.015;   // 已标定的阻力系数
params.n = 2.0;     // 已标定的速度指数
params.m = 0.1;     // 小球质量（公斤）
params.deltaH = 0.5; // 炮口与目标高度差（米）- 只考虑高度差，不考虑绝对高度
params.thetaMax = Math.PI / 4; // 最大仰角（45度）
solver.setParameters(params);

// 3. 直接输入相对位置和小车速度
// 例如：目标在小车正前方 3 米，右侧 2 米，小车以 0.5 m/s 向前移动
double relativeX = 3.0;  // 目标相对小车的 x 坐标
double relativeY = 2.0;  // 目标相对小车的 y 坐标
double robotVx = 0.5;    // 小车在 x 方向的速度
double robotVy = 0.0;    // 小车在 y 方向的速度

// 4. 求解
Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy);

// 5. 处理结果
if (result.success) {
    double turretAngle = result.getTurretPhiDegrees(); // 炮塔水平朝向（度）
    double elevationAngle = result.getThetaDegrees();   // 炮管仰角（度）
    double flightTime = result.flightTime;             // 飞行时间（秒）
    
    System.out.println("炮塔朝向: " + turretAngle + " 度");
    System.out.println("炮管仰角: " + elevationAngle + " 度");
    System.out.println("飞行时间: " + flightTime + " 秒");
} else {
    System.out.println("求解失败: " + result.message);
}
```

### 3.2 实时控制

**功能**：在 FTC 比赛中实时更新目标角度。

**实现建议**：

1. **传感器数据**：使用 Limelight 等视觉传感器获取目标位置和速度
2. **更新频率**：建议 20-50Hz 更新一次
3. **状态管理**：维护机器人和目标的状态

**代码示例**：

```java
// 在 OpMode 中使用 - 完整参数版
@Override
public void runOpMode() {
    // 初始化
    Solver solver = new Solver();
    ProjectileParameters params = new ProjectileParameters();
    // 设置参数...
    solver.setParameters(params);
    
    Limelight limelight = hardwareMap.get(Limelight3A.class, "limelight");
    
    while (opModeIsActive()) {
        // 1. 获取视觉数据
        double targetX = getTargetXFromLimelight(limelight);
        double targetY = getTargetYFromLimelight(limelight);
        double targetVx = getTargetVxFromLimelight(limelight);
        double targetVy = getTargetVyFromLimelight(limelight);
        
        // 2. 获取机器人状态
        double robotX = getRobotX();
        double robotY = getRobotY();
        double robotVx = getRobotVx();
        double robotVy = getRobotVy();
        
        // 3. 构建状态对象
        RobotState robot = new RobotState(robotX, robotY, robotVx, robotVy);
        TargetPredictor.TargetState target = new TargetPredictor.TargetState(targetX, targetY, targetVx, targetVy);
        
        // 4. 求解
        Solver.SolverResult result = solver.solve(target.x, target.y, robot, target);
        
        // 5. 控制炮塔
        if (result.success) {
            double turretAngle = result.getTurretPhiDegrees();
            double elevationAngle = result.getThetaDegrees();
            
            setTurretAngle(turretAngle);
            setElevationAngle(elevationAngle);
            
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Elevation Angle", elevationAngle);
        }
        
        telemetry.update();
    }
}
```

**简化参数版（推荐）**：

```java
// 在 OpMode 中使用 - 简化参数版
@Override
public void runOpMode() {
    // 初始化
    Solver solver = new Solver();
    ProjectileParameters params = new ProjectileParameters();
    // 设置标定后的参数
    params.v0 = 6.2;
    params.k = 0.012;
    params.n = 1.9;
    params.m = 0.1;
    params.deltaH = 0.45; // 炮口与目标高度差
    params.thetaMax = Math.PI / 4;
    solver.setParameters(params);
    
    Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
    
    telemetry.addLine("准备就绪，按 START 开始");
    telemetry.update();
    
    waitForStart();
    
    while (opModeIsActive()) {
        // 1. 从 Limelight 获取目标相对位置
        double relativeX = getTargetRelativeX(limelight); // 目标相对小车的 x 坐标
        double relativeY = getTargetRelativeY(limelight); // 目标相对小车的 y 坐标
        
        // 2. 获取小车速度（例如从编码器或里程计）
        double robotVx = getRobotVelocityX(); // 小车在 x 方向的速度
        double robotVy = getRobotVelocityY(); // 小车在 y 方向的速度
        
        // 3. 直接求解（目标静止，速度为 0）
        Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy);
        
        // 4. 控制炮塔
        if (result.success) {
            double turretAngle = result.getTurretPhiDegrees();
            double elevationAngle = result.getThetaDegrees();
            
            setTurretAngle(turretAngle);
            setElevationAngle(elevationAngle);
            
            telemetry.addData("Turret Angle", turretAngle);
            telemetry.addData("Elevation Angle", elevationAngle);
        } else {
            telemetry.addLine("求解失败: " + result.message);
        }
        
        telemetry.update();
    }
}

private double getTargetRelativeX(Limelight3A limelight) {
    // 从 Limelight 获取目标相对 x 坐标
    // 这里需要根据实际硬件和坐标系转换计算
    return 3.0; // 示例值
}

private double getTargetRelativeY(Limelight3A limelight) {
    // 从 Limelight 获取目标相对 y 坐标
    return 2.0; // 示例值
}

private double getRobotVelocityX() {
    // 从编码器或里程计获取小车 x 方向速度
    return 0.5; // 示例值
}

private double getRobotVelocityY() {
    // 从编码器或里程计获取小车 y 方向速度
    return 0.0; // 示例值
}
```

### 3.3 性能优化

**优化策略**：

1. **降低计算精度**：在实时控制中，可减少 RK4 积分步长和二分搜索迭代次数
2. **缓存结果**：如果目标和机器人状态变化不大，可缓存上一次的求解结果
3. **多线程处理**：在 Control Hub 上可考虑使用多线程进行计算

**代码优化**：

```java
// 优化后的求解器配置
Solver solver = new Solver();
solver.setMaxOuterIterations(3);        // 减少外层迭代次数
solver.setMaxBinarySearchIterations(10); // 减少二分搜索次数

// 缓存机制
Solver.SolverResult lastResult = null;
double lastTargetX, lastTargetY;
double lastRobotVx, lastRobotVy;

// 仅在状态变化时重新计算
if (Math.abs(targetX - lastTargetX) > 0.1 || 
    Math.abs(targetY - lastTargetY) > 0.1 ||
    Math.abs(robotVx - lastRobotVx) > 0.1 ||
    Math.abs(robotVy - lastRobotVy) > 0.1) {
    lastResult = solver.solve(targetX, targetY, robot, target);
    lastTargetX = targetX;
    lastTargetY = targetY;
    lastRobotVx = robotVx;
    lastRobotVy = robotVy;
}
```

## 4. 常见问题与解决方案

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 求解失败 | 目标超出射程 | 减小目标距离或调整发射参数 |
| 命中率低 | 阻力参数不准确 | 重新标定阻力参数 k 和 n |
| 实时性差 | 计算开销大 | 优化求解器参数，减少迭代次数 |
| 角度抖动 | 目标位置数据噪声 | 对视觉数据进行低通滤波 |
| 侧向偏差 | 机器人速度测量不准 | 校准机器人速度传感器 |
| 落点偏差大 | 轨迹插值计算错误 | 已修复：插值时应使用上一状态的位置坐标而非高度坐标 |

## 5. 代码示例

### 5.1 完整的参数标定示例

```java
public class CalibrationOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("准备进行参数标定");
        telemetry.update();
        
        waitForStart();
        
        // 1. 标定初速度
        CalibrationHelper calibrator = new CalibrationHelper();
        ProjectileParameters params = new ProjectileParameters();
        
        double measuredRange = 2.8; // 实际测量的水平距离
        double v0 = calibrator.calibrateV0(measuredRange, params.deltaH, params);
        
        telemetry.addData("初速度 v0", v0);
        telemetry.update();
        sleep(2000);
        
        // 2. 拟合阻力参数
        double[][] rangeAngleData = {
            {1.5, Math.toRadians(12)},
            {2.0, Math.toRadians(17)},
            {2.5, Math.toRadians(22)},
            {3.0, Math.toRadians(28)},
            {3.5, Math.toRadians(35)}
        };
        
        params.v0 = v0;
        CalibrationHelper.CalibrationData dragResult = calibrator.fitDragParameters(rangeAngleData, params);
        
        telemetry.addData("阻力系数 k", dragResult.k);
        telemetry.addData("速度指数 n", dragResult.n);
        telemetry.addData("拟合误差", dragResult.totalError);
        telemetry.update();
        
        sleep(5000);
    }
}
```

### 5.2 实时预测示例（使用简化参数）

```java
@TeleOp(name = "ShootingController", group = "Competition")
public class ShootingController extends LinearOpMode {
    private Solver solver;
    private ProjectileParameters params;
    private Limelight3A limelight;
    
    @Override
    public void runOpMode() {
        // 初始化
        solver = new Solver();
        params = new ProjectileParameters();
        // 设置标定后的参数
        params.v0 = 6.2;     // 初速度（米/秒）
        params.k = 0.012;    // 阻力系数
        params.n = 1.9;      // 速度指数
        params.m = 0.1;      // 小球质量（公斤）
        params.deltaH = 0.45; // 炮口与目标高度差（米）- 只考虑高度差，不考虑绝对高度
        params.thetaMax = Math.PI / 4; // 最大仰角（45度）
        solver.setParameters(params);
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        telemetry.addLine("准备就绪，按 START 开始");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // 1. 获取目标相对位置
            double relativeX = getTargetRelativeX(); // 目标相对小车的 x 坐标
            double relativeY = getTargetRelativeY(); // 目标相对小车的 y 坐标
            
            // 2. 获取小车速度
            double robotVx = getRobotVelocityX(); // 小车在 x 方向的速度
            double robotVy = getRobotVelocityY(); // 小车在 y 方向的速度
            
            // 3. 直接求解（目标静止，使用简化参数）
            Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy);
            
            // 4. 显示结果
            if (result.success) {
                telemetry.addData("炮塔朝向", result.getTurretPhiDegrees());
                telemetry.addData("炮管仰角", result.getThetaDegrees());
                telemetry.addData("飞行时间", result.flightTime);
            } else {
                telemetry.addLine("求解失败: " + result.message);
            }
            
            telemetry.update();
        }
    }
    
    private double getTargetRelativeX() {
        // 从 Limelight 获取目标相对 x 坐标
        // 小车正前方为 x 正方向
        // 这里需要根据实际硬件和坐标系转换计算
        return 3.0; // 示例值
    }
    
    private double getTargetRelativeY() {
        // 从 Limelight 获取目标相对 y 坐标
        return 2.0; // 示例值
    }
    
    private double getRobotVelocityX() {
        // 从编码器或里程计获取小车 x 方向速度
        return 0.5; // 示例值
    }
    
    private double getRobotVelocityY() {
        // 从编码器或里程计获取小车 y 方向速度
        return 0.0; // 示例值
    }
}
```

## 6. 总结

RK4 跑打算法通过考虑小车速度、目标运动和空气阻力，能够精确计算炮台角度，提高命中率。使用时需要：

1. **首先进行参数标定**：测定初速度 v0 和阻力参数 k、n
2. **设置合理的物理参数**：根据实际硬件情况调整
3. **实时更新状态**：持续获取机器人和目标的最新状态
4. **优化性能**：根据硬件能力调整计算精度

通过正确使用本文档中的方法，您的机器人将能够在 FTC 比赛中实现精确的跑打功能。