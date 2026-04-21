# Library 包说明

## 包概述

Library 包提供了一系列用于 FTC 机器人电机控制的工具类，包括电机速度控制、SVA 参数调优等功能。这些工具类旨在简化电机控制的实现，提高控制精度和稳定性。

## 包含的类

### 1. ExampleVoltageOutMotor

**功能**：实现使用电压输出的电机速度控制，结合 PIDSVAController 实现高精度速度控制。

**代码分解**：
- **成员变量**：
  - 静态参数：kP、kI、kD、maxI、kS、kV、kA、outputMin、outputMax（支持 Dashboard 热调参）
  - 实例变量：motor（电机实例）、voltageOut（电压输出控制器）、controller（PIDSVAController 实例）、config（控制器配置）、telemetry（遥测实例）、targetVelocity（目标速度）、lastUpdateTime（上次更新时间）

- **构造函数**：
  - 初始化电机并配置运行模式
  - 初始化电压输出控制器
  - 初始化 PIDSVAController 及其配置

- **主要方法**：
  - `setTargetVelocity(double velocity)`：设置目标速度
  - `update()`：更新电机控制，包括同步参数、计算时间间隔、获取当前速度、计算控制器输出、转换为功率并设置给电机、输出遥测数据
  - `stop()`：停止电机

**电机调用方法**：
1. 创建 ExampleVoltageOutMotor 实例：`ExampleVoltageOutMotor motor = new ExampleVoltageOutMotor(hardwareMap, "motorName", telemetry);`
2. 设置目标速度：`motor.setTargetVelocity(targetVelocity);`
3. 在循环中调用 update() 方法：`motor.update();`
4. 停止电机：`motor.stop();`

### 2. MotorSVATuning

**功能**：用于自动或手动调优电机的 SVA 参数（kS、kV、kA）。

**代码分解**：
- **成员变量**：
  - 实例变量：motor（电机实例）、fileWriter（文件写入器）、logFileName（日志文件名）、voltageOut（电压输出控制器）、lastA（上次A按钮状态）
  - 枚举：TuningMode（调优模式：AUTOMATIC、MANUAL）、State（调优状态）
  - 状态变量：state（当前调优状态）、points_kS_kV（用于拟合kS和kV的数据点）、points_kA（用于拟合kA的数据点）、meanFilter（均值滤波器）
  - 静态参数：kS、kV、kA（调优结果）、tuningMode、KS_VOLTAGE_INCREMENT、KS_VELOCITY_THRESHOLD、testPoints、maxVariance、kA_TestVoltages、voltageIncrement、motorName

- **主要方法**：
  - `runOpMode()`：主操作模式，包括初始化电机、电压输出控制器、日志文件，以及调优主循环
  - 调优流程：kS评估 → kS_kV拟合 → 等待电机停止 → kA评估 → 调优完成

**电机调用方法**：
1. 在 FTC Driver Station 中选择并运行 "Motor SVA Tuning" OpMode
2. 在初始化阶段按A键切换调优模式（自动/手动）
3. 自动模式：系统自动完成调优过程
4. 手动模式：使用方向键调整电压，按A键记录数据点，按Start键结束调优
5. 调优完成后，记录的kS、kV、kA参数可用于 ExampleVoltageOutMotor

### 3. ExampleVelPIDSVAMotor

**功能**：示例 TeleOp，展示如何使用 ExampleVoltageOutMotor 控制电机速度。

**代码分解**：
- **成员变量**：
  - 静态参数：targetVelocity（目标速度）

- **主要方法**：
  - `runOpMode()`：主操作模式，创建 ExampleVoltageOutMotor 实例，使用左摇杆控制目标速度，按A键停止电机

**电机调用方法**：
1. 在 FTC Driver Station 中选择并运行 "Example VoltageOut Motor" OpMode
2. 使用左摇杆Y轴控制电机速度（上推增加速度，下拉减少速度）
3. 按A键停止电机

## 工作原理

### 电压输出控制

Library 包中的电机控制基于电压输出，而非直接的功率控制。这种方法的优势在于：
- 不受电池电压波动的影响
- 提供更稳定的电机性能
- 提高控制精度

### SVA 前馈控制

SVA 前馈控制通过以下公式计算前馈项：

```
SVA = kS * sign(velocity) + kV * velocity + kA * acceleration
```

- `kS`：静态摩擦系数，用于克服静摩擦力
- `kV`：速度系数，与速度成正比
- `kA`：加速度系数，与加速度成正比

### PID 反馈控制

PID 反馈控制通过计算目标值与实际值之间的误差，根据比例、积分、微分三个环节进行控制：

- **比例环节**：与当前误差成正比，提供即时响应
- **积分环节**：与误差的积分成正比，消除稳态误差
- **微分环节**：与误差的变化率成正比，预测误差趋势，提高稳定性

### 复合控制

ExampleVoltageOutMotor 结合了 PID 反馈控制和 SVA 前馈控制，通过前馈项提前补偿系统的动态特性，减少 PID 控制器的负担，提高控制精度和响应速度。

## 使用示例

### 基本电机速度控制

```
java
// 创建电机控制器
ExampleVoltageOutMotor motor = new ExampleVoltageOutMotor(hardwareMap, "motorName", telemetry);

// 设置目标速度
motor.setTargetVelocity(1000); // 1000 ticks per second

// 在循环中更新控制
while (opModeIsActive()) {
    motor.update();
    telemetry.update();
}

// 停止电机
motor.stop();
```

### SVA 参数调优

1. 运行 "Motor SVA Tuning" OpMode
2. 选择自动或手动模式
3. 完成调优后，记录得到的 kS、kV、kA 参数
4. 将这些参数应用到 ExampleVoltageOutMotor 的静态变量中

### 游戏手柄控制电机速度

运行 "Example VoltageOut Motor" OpMode，使用左摇杆控制电机速度，按A键停止电机。

## 注意事项

1. **电机配置**：确保电机名称与硬件配置文件中的名称一致
2. **参数调优**：在使用 ExampleVoltageOutMotor 前，建议先使用 MotorSVATuning 调优 SVA 参数
3. **电压输出**：确保电池电压充足，以获得最佳控制效果
4. **遥测数据**：通过遥测数据监控电机速度和控制器输出，以便调整参数
5. **热调参**：可以通过 FTC Dashboard 实时调整 PID 和 SVA 参数，无需重新编译代码

## 总结

Library 包提供了一套完整的电机控制解决方案，包括：
- 高精度的速度控制（ExampleVoltageOutMotor）
- 自动化的 SVA 参数调优（MotorSVATuning）
- 示例 TeleOp（ExampleVelPIDSVAMotor）

通过合理使用这些工具类，可以实现更加稳定、精确的电机控制，提高机器人的整体性能。