# RK4 跑打算法使用指南

本文档详细说明如何使用 RK4 跑打算法代码测定参数和预测炮台角度，以实现精确的目标命中。

## 目录

- [1. 系统架构](#1-系统架构)
- [1.1 类依赖关系](#11-类依赖关系)
- [2. 参数标定流程](#2-参数标定流程)
  - [2.1 快速标定（推荐）](#21-快速标定推荐)
  - [2.2 手动标定](#22-手动标定)
  - [2.3 初速度 v0 测定](#23-初速度-v0-测定)
  - [2.4 阻力参数 kx 和 ky 测定](#24-阻力参数-kx-和-ky-测定)
- [3. 预测角度使用方法](#3-预测角度使用方法)
  - [3.0 高度差处理说明](#30-高度差处理说明)
  - [3.1 基本使用](#31-基本使用)
  - [3.2 实时控制](#32-实时控制)
  - [3.3 性能优化](#33-性能优化)
  - [3.4 多参数配置](#34-多参数配置)
  - [3.5 模式相关范围设置](#35-模式相关范围设置)
- [4. 常见问题与解决方案](#4-常见问题与解决方案)
- [5. 自动模式选择](#5-自动模式选择)
  - [5.1 AutoSelect 类](#51-autoselect-类)
  - [5.2 使用示例](#52-使用示例)
  - [5.3 自定义配置](#53-自定义配置)
  - [5.4 注意事项](#54-注意事项)
  - [5.5 完整 OpMode 示例](#55-完整-opmode-示例)
- [6. 总结](#6-总结)

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
| **ProjectileParameters** | 无 | 物理参数（v₀, kx, ky, m, g, Δh） |
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
   - 阻力和升力参数数据：在不同距离设置靶标，调整仰角直到命中，记录 `(距离, 仰角)` 数据对

2. **准备 CSV 文件**：
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
- 初速度（米/秒）
- 炮口高度（米）
- 小球质量（公斤）

#### 输出结果

程序会输出：
```
=====================================
===== 标定结果 =====
初速度 (v0): 7.500 m/s (手动设置)
阻力系数 (kx): 0.012500
升力系数 (ky): 0.005000
拟合总误差: 0.001234
均方根误差: 0.015811
阻力参数数据点: 5
状态: 成功

===== 推荐代码 =====
params.v0 = 7.500; // 手动设置的初速度
params.kx = 0.012500;
params.ky = 0.005000;
=====================================
```

### 2.2 手动设置初速度

如果无法使用应用程序，可以手动设置初速度。

**目的**：设置小球离开炮口时的初速度。

**步骤**：

1. **确定初速度**：
   - 通过制造商提供的规格
   - 或通过简单的测试估计
   - 或使用之前的经验值

2. **代码实现**：

```java
// 创建默认参数
ProjectileParameters params = new ProjectileParameters();

// 手动设置初速度
params.v0 = 7.5; // 初速度（米/秒）
params.deltaH = 0.2; // 炮口高度

// 输出结果
System.out.println("手动设置的初速度: " + params.v0 + " m/s");
```

### 2.4 阻力参数 kx 和 ky 测定

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
System.out.println("kx: " + result.k);
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
// 2.1 基本用法：单参数配置
ProjectileParameters params = new ProjectileParameters();
params.v0 = 6.0;    // 已标定的初速度
params.kx = 0.015;   // 已标定的阻力系数
params.ky = 0.005;   // 已标定的升力系数
params.m = 0.1;     // 小球质量（公斤）
params.deltaH = 0.5; // 炮口与目标高度差（米）- 只考虑高度差，不考虑绝对高度
params.thetaMax = Math.PI / 4; // 最大仰角（45度）
solver.setParameters(params);

// 3. 准备输入数据
RobotState robot = new RobotState(0, 0, 0.5, 0.3); // (x,y,vx,vy)
TargetPredictor.TargetState target = new TargetPredictor.TargetState(3, 2, 0, 0.5); // (x,y,vx,vy)

// 4. 求解 - 使用默认初速度
Solver.SolverResult result = solver.solve(target.x, target.y, robot, target);

// 4. 求解 - 指定初速度（新功能）
double customV0 = 7.5; // 自定义初速度
Solver.SolverResult resultWithCustomV0 = solver.solve(target.x, target.y, robot, target, customV0);

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
// 2.1 基本用法：单参数配置
ProjectileParameters params = new ProjectileParameters();
params.v0 = 6.0;    // 已标定的初速度
params.kx = 0.015;   // 已标定的阻力系数
params.ky = 0.005;   // 已标定的升力系数
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
params.kx = 0.012;
params.ky = 0.003;
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
        
        // 3. 直接求解（目标静止，速度为 0）- 使用默认初速度
        Solver.SolverResult result = solver.solve(relativeX, relativeY, robotVx, robotVy);
        
        // 3. 直接求解 - 指定初速度（新功能）
        double customV0 = 7.5; // 自定义初速度
        Solver.SolverResult resultWithCustomV0 = solver.solve(relativeX, relativeY, robotVx, robotVy, customV0);
        
        // 4. 直接求解 - 指定固定仰角，计算所需初速度（新功能）
        double fixedTheta = Math.toRadians(30); // 固定仰角 30 度
        Solver.SolverResult resultWithFixedTheta = solver.solve(relativeX, relativeY, robotVx, robotVy, fixedTheta, "Vel");
        System.out.println("固定仰角: " + resultWithFixedTheta.getThetaDegrees() + "度"); // 显示固定仰角
        System.out.println("所需初速度: " + resultWithFixedTheta.getV0() + " m/s"); // 显示计算出的初速度
        
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

// 在主循环中使用缓存
while (opModeIsActive()) {
    double targetX = getTargetX();
    double targetY = getTargetY();
    double robotVx = getRobotVx();
    double robotVy = getRobotVy();
    
    // 检查状态是否变化不大
    if (lastResult != null &&
        Math.abs(targetX - lastTargetX) < 0.1 &&
        Math.abs(targetY - lastTargetY) < 0.1 &&
        Math.abs(robotVx - lastRobotVx) < 0.1 &&
        Math.abs(robotVy - lastRobotVy) < 0.1) {
        // 使用缓存结果
        useSolverResult(lastResult);
    } else {
        // 重新求解
        Solver.SolverResult result = solver.solve(targetX, targetY, robotVx, robotVy);
        lastResult = result;
        lastTargetX = targetX;
        lastTargetY = targetY;
        lastRobotVx = robotVx;
        lastRobotVy = robotVy;
        useSolverResult(result);
    }
}
```

### 3.4 多参数配置

**功能**：支持管理和切换多种参数配置，适应不同的发射条件。

**代码示例**：
```java
// 创建不同参数集
ProjectileParameters normalParams = new ProjectileParameters(6.0, 0.015, 0.005, 0.1, 0.5, Math.PI / 4);
ProjectileParameters highPowerParams = new ProjectileParameters(7.0, 0.018, 0.006, 0.1, 0.5, Math.PI / 3);
ProjectileParameters lowPowerParams = new ProjectileParameters(5.0, 0.012, 0.004, 0.1, 0.5, Math.PI / 4);

// 添加参数集到求解器
solver.addParameterSet("normal", normalParams);
solver.addParameterSet("highPower", highPowerParams);
solver.addParameterSet("lowPower", lowPowerParams);

// 切换参数集
solver.switchParameterSet("highPower");
System.out.println("当前参数集: " + solver.getCurrentParameterSetName());

// 获取当前参数
ProjectileParameters currentParams = solver.getCurrentParameters();
```

**使用场景**：
- 不同天气条件（温度、湿度影响空气阻力）
- 不同场地（室内/室外）
- 不同小球类型（重量、表面材质）
- 不同发射模式（高功率/低功率）

**最佳实践**：
- 为每种常见场景创建独立的参数集
- 在比赛前根据实际条件选择合适的参数集
- 使用参数标定工具为每种场景生成准确的参数

### 3.5 模式相关范围设置

**功能**：设置 Yaw 模式和 Vel 模式下的仰角范围，以及初速度范围。

**默认范围**：
- Yaw 模式（由初速度求仰角）：仰角范围 0-55 度
- Vel 模式（由仰角求初速度）：仰角范围 0-55 度
- 初速度范围：2.0-23.0 m/s

**代码示例**：
```java
// 创建求解器
Solver solver = new Solver();

// 自定义 Yaw 模式仰角范围（例如 0-40 度）
solver.setYawModeThetaRange(0, Math.toRadians(40));

// 自定义 Vel 模式仰角范围（例如 0-60 度）
solver.setVelModeThetaRange(0, Math.toRadians(60));

// 自定义初速度范围（例如 3.0-12.0 m/s）
solver.setV0Range(3.0, 12.0);

// 获取当前范围设置
double[] yawRange = solver.getYawModeThetaRange();
double[] velRange = solver.getVelModeThetaRange();
double[] v0Range = solver.getV0Range();

System.out.println("Yaw 模式仰角范围: " + Math.toDegrees(yawRange[0]) + "-" + Math.toDegrees(yawRange[1]) + " 度");
System.out.println("Vel 模式仰角范围: " + Math.toDegrees(velRange[0]) + "-" + Math.toDegrees(velRange[1]) + " 度");
System.out.println("初速度范围: " + v0Range[0] + "-" + v0Range[1] + " m/s");
```

**使用建议**：
- 根据硬件能力设置合理的仰角范围，避免超出机械极限
- 根据发射系统的实际能力设置初速度范围
- 在不同场景下调整范围以获得最佳性能


## 4. 常见问题与解决方案

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 求解失败 | 目标超出射程 | 减小目标距离或调整发射参数 |
| 命中率低 | 阻力参数不准确 | 重新标定阻力参数 kx 和 ky |
| 实时性差 | 计算开销大 | 优化求解器参数，减少迭代次数 |
| 角度抖动 | 目标位置数据噪声 | 对视觉数据进行低通滤波 |

## 5. 自动模式选择

### 5.1 AutoSelect 类

`AutoSelect` 类提供了自动选择最优初速度和仰角的功能，现在只支持一种合并模式：

- **合并模式**：同时输入初始初速度和仰角值，使用二分法算法同时优化初速度和仰角

### 5.2 使用示例

#### 基本使用

```java
// 1. 创建 AutoSelect 实例
AutoSelect autoSelect = new AutoSelect();

// 2. 准备输入数据
double relativeX = 3.0; // 目标相对 x 坐标
double relativeY = 2.0; // 目标相对 y 坐标
double robotVx = 0.5;   // 小车 x 方向速度
double robotVy = 0.3;   // 小车 y 方向速度

// 3. 使用合并模式
// 同时输入初始初速度和仰角
double initialV0 = 8.0;     // 初始初速度（m/s）
double initialTheta = Math.toRadians(55); // 初始仰角（55度）
AutoSelect.AutoSelectResult result = autoSelect.Select(relativeX, relativeY, robotVx, robotVy, initialV0, initialTheta);

// 4. 处理结果
if (result.success) {
    System.out.println("合并模式成功:");
    System.out.println("  仰角: " + result.getThetaDegrees() + "度");
    System.out.println("  初速度: " + result.v0 + " m/s");
    System.out.println("  旋转角: " + result.getTurretPhiDegrees() + "度");
    System.out.println("  消息: " + result.message);
} else {
    System.out.println("合并模式失败: " + result.message);
}
```

#### 构造时设定范围

```java
// 1. 创建物理参数
ProjectileParameters params = new ProjectileParameters();
params.v0 = 8.0;     // 初速度
params.kx = 0.0004;   // 阻力系数
params.ky = 0.0001;   // 升力系数
params.m = 0.1;      // 小球质量
params.deltaH = 1.0; // 高度差

// 2. 创建 AutoSelect 实例并设定范围
// 参数：物理参数, 初速度最小值, 初速度最大值, 仰角最小值, 仰角最大值
AutoSelect autoSelect = new AutoSelect(
    params, 
    3.0,  // 最小初速度 (m/s)
    9.0,  // 最大初速度 (m/s)
    Math.toRadians(40),  // 最小仰角 (40度)
    Math.toRadians(60)   // 最大仰角 (60度)
);

// 3. 使用合并模式
// 同时输入初始初速度和仰角
double initialV0 = 6.0;     // 初始初速度（m/s）
double initialTheta = Math.toRadians(50); // 初始仰角（50度）
AutoSelect.AutoSelectResult result = autoSelect.Select(relativeX, relativeY, robotVx, robotVy, initialV0, initialTheta);
```

### 5.3 自定义配置

#### 方法一：使用构造时设定范围

```java
// 创建 AutoSelect 实例并设定范围
// 参数：物理参数, 初速度最小值, 初速度最大值, 仰角最小值, 仰角最大值
AutoSelect autoSelect = new AutoSelect(
    params, 
    3.0,  // 最小初速度 (m/s)
    9.0,  // 最大初速度 (m/s)
    Math.toRadians(40),  // 最小仰角 (40度)
    Math.toRadians(60)   // 最大仰角 (60度)
);
```

#### 方法二：使用 setter 方法设定范围

```java
// 自定义最优初速度列表
List<Double> customV0List = Arrays.asList(7.0, 7.5, 8.0, 8.5, 9.0);
autoSelect.setOptimalV0List(customV0List);

// 自定义最优仰角列表（弧度）
List<Double> customThetaList = Arrays.asList(
    Math.toRadians(45),
    Math.toRadians(55),
    Math.toRadians(65)
);
autoSelect.setOptimalThetaList(customThetaList);

// 自定义初速度范围
autoSelect.setV0Range(3.0, 9.0); // 最小 3.0 m/s，最大 9.0 m/s

// 自定义仰角范围（弧度）
autoSelect.setThetaRange(Math.toRadians(40), Math.toRadians(60)); // 40-60 度
```

### 5.4 注意事项

1. **参数范围**：
   - 初速度范围：可在构造时自定义（默认 0.0 - 10.0 m/s）
   - 仰角范围：可在构造时自定义（默认 45 - 65度）
   - 超出范围的参数将被拒绝

2. **初始值选择**：
   - 初始初速度应选择在硬件允许的范围内
   - 初始仰角应选择在设定的仰角范围内

3. **优化算法**：
   - 使用二分法同时优化初速度和仰角
   - 扩大搜索范围以找到全局最优解
   - 增加迭代次数以提高精度
   - 最终验证误差小于0.1m才返回结果

4. **性能考虑**：
   - 优化过程可能需要更多计算时间
   - 建议在非实时控制时使用，或在硬件性能足够的情况下使用

### 5.5 完整 OpMode 示例

以下是一个完整的 FTC OpMode 示例，展示如何在比赛中使用 RK4 跑打算法：

```java
@TeleOp(name = "ShootingController", group = "Competition")
public class ShootingController extends LinearOpMode {
    private Solver solver;
    private ProjectileParameters params;
    private Limelight3A limelight;
    private AutoSelect autoSelect;
    
    @Override
    public void runOpMode() {
        // 初始化求解器
        solver = new Solver();
        
        // 设置物理参数（使用标定结果）
params = new ProjectileParameters();
params.v0 = 8.0;     // 初速度（米/秒）
params.kx = 0.0004;   // 阻力系数
params.ky = 0.0001;   // 升力系数
params.m = 0.1;      // 小球质量（公斤）
params.deltaH = 1.0; // 炮口与目标高度差（米）
solver.setParameters(params);
        
        // 初始化 Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        // 初始化AutoSelect（带自定义范围）
        // 参数：物理参数, 初速度最小值, 初速度最大值, 仰角最小值, 仰角最大值
        autoSelect = new AutoSelect(
            params, 
            3.0,  // 最小初速度 (m/s)
            9.0,  // 最大初速度 (m/s)
            Math.toRadians(40),  // 最小仰角 (40度)
            Math.toRadians(60)   // 最大仰角 (60度)
        );
        
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
            
            // 3. 使用合并模式
            double initialV0 = 8.0;     // 初始初速度（m/s）
            double initialTheta = Math.toRadians(55); // 初始仰角（55度）
            AutoSelect.AutoSelectResult result = autoSelect.Select(relativeX, relativeY, robotVx, robotVy, initialV0, initialTheta);
            
            // 4. 显示结果
            if (result.success) {
                telemetry.addData("仰角", result.getThetaDegrees());
                telemetry.addData("初速度", result.v0);
                telemetry.addData("旋转角", result.getTurretPhiDegrees());
                telemetry.addData("状态", result.message);
                
                // 控制炮塔
                setTurretAngle(result.getTurretPhiDegrees());
                setElevationAngle(result.getThetaDegrees());
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
    
    private void setTurretAngle(double angle) {
        // 控制炮塔旋转到指定角度
    }
    
    private void setElevationAngle(double angle) {
        // 控制炮管仰角到指定角度
    }
}
```

## 6. 总结

RK4 跑打算法通过考虑小车速度、目标运动和空气阻力，能够精确计算炮台角度，提高命中率。使用时需要：

1. **首先进行参数标定**：测定初速度 v0 和阻力参数 kx、ky
2. **设置合理的物理参数**：根据实际硬件情况调整
3. **实时更新状态**：持续获取机器人和目标的最新状态
4. **优化性能**：根据硬件能力调整计算精度

通过正确使用本文档中的方法，您的机器人将能够在 FTC 比赛中实现精确的跑打功能。