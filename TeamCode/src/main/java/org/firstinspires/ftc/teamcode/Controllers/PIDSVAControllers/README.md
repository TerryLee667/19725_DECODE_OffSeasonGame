# PIDSVAControllers 包说明

## 包概述

PIDSVAControllers 包提供了一系列用于 FTC 机器人控制的控制器实现，包括基本 PID 控制器、SVA 前馈控制器以及结合两者的 PIDSVAController。这些控制器可用于机器人的各种运动控制场景，如电机速度控制、位置控制等。

## 包含的类

### 1. SlotConfig

**功能**：配置类，用于设置 PID 和 SVA 控制器的参数。

**主要属性**：
- `kP`：比例系数
- `kI`：积分系数
- `kD`：微分系数
- `maxI`：积分上限
- `kS`：静态摩擦系数
- `kV`：速度系数
- `kA`：加速度系数
- `outputMin`：输出最小值
- `outputMax`：输出最大值

**使用方法**：采用建造者模式，支持链式调用设置参数。

```java
SlotConfig config = new SlotConfig()
    .withKP(0.1)
    .withKI(0.01)
    .withKD(0.05)
    .withMaxI(1.0)
    .withKS(0.05)
    .withKV(0.1)
    .withKA(0.01)
    .withOutputLimits(-1.0, 1.0);
```

### 2. PIDController

**功能**：实现基本的 PID 控制器功能，用于根据目标值和测量值计算控制输出。

**主要方法**：
- `calculate(double setpoint, double measurement, double dt)`：计算 PID 输出
- `reset()`：重置控制器状态
- `setPID(double kP, double kI, double kD)`：设置 PID 参数
- `setMaxI(double maxI)`：设置积分上限

**使用场景**：适用于需要基本 PID 控制的场景，如简单的位置控制或速度控制。

### 3. SVAController

**功能**：实现 SVA（静态摩擦、速度、加速度）前馈控制器，用于计算电机控制的前馈项。

**主要方法**：
- `calculate(double velocity, double acceleration)`：计算 SVA 前馈输出
- `setSVA(double kS, double kV, double kA)`：设置 SVA 参数

**使用场景**：适用于需要前馈控制的场景，可与 PID 控制器配合使用以提高控制精度。

### 4. PIDSVAController

**功能**：实现带有 SVA 前馈的 PID 控制器，支持多 slot 配置，可根据不同场景切换参数。

**主要方法**：
- `withSlot0(SlotConfig config)`：设置默认 slot(0号) 的参数
- `withSlot(int slot, SlotConfig config)`：设置指定 slot 的参数
- `setSlot(int slot)`：切换到指定 slot
- `calculate(double setpoint, double measurement, double dt)`：简单速度闭环
- `calculate(double setpoint, double measurement, double dt, boolean VelCycle)`：速度闭环或位置闭环
- `calculate(double setpoint, double measurement, double velocity, double acceleration, double dt)`：完整 PIDSVA 闭环
- `reset()`：重置控制器状态

**使用场景**：适用于复杂的控制场景，如需要在不同速度范围使用不同参数的电机控制。

## 控制器工作原理

### PID 控制

PID 控制器通过计算目标值与实际值之间的误差，根据比例、积分、微分三个环节进行控制：

- **比例环节**：与当前误差成正比，提供即时响应
- **积分环节**：与误差的积分成正比，消除稳态误差
- **微分环节**：与误差的变化率成正比，预测误差趋势，提高稳定性

### SVA 前馈控制

SVA 前馈控制器通过以下公式计算前馈项：

```
SVA = kS * sign(velocity) + kV * velocity + kA * acceleration
```

- `kS`：静态摩擦系数，用于克服静摩擦力
- `kV`：速度系数，与速度成正比
- `kA`：加速度系数，与加速度成正比

### PID+SVA 复合控制

PIDSVAController 将 PID 反馈控制与 SVA 前馈控制相结合，通过前馈项提前补偿系统的动态特性，减少 PID 控制器的负担，提高控制精度和响应速度。

## 使用示例

### 基本 PID 控制

```
java
// 创建 PID 控制器
PIDController pidController = new PIDController(0.1, 0.01, 0.05);

// 计算输出
double output = pidController.calculate(targetPosition, currentPosition, 0.02);

// 重置控制器
pidController.reset();
```

### SVA 前馈控制

```java
// 创建 SVA 控制器
SVAController svaController = new SVAController(0.05, 0.1, 0.01);

// 计算前馈输出
double feedforward = svaController.calculate(currentVelocity, currentAcceleration);
```

### PID+SVA 复合控制

```java
// 创建配置
SlotConfig config = new SlotConfig()
    .withKP(0.1)
    .withKI(0.01)
    .withKD(0.05)
    .withKS(0.05)
    .withKV(0.1)
    .withKA(0.01);

// 创建 PIDSVAController
PIDSVAController controller = new PIDSVAController().withSlot0(config);

// 计算输出（速度闭环）
double output = controller.calculate(targetVelocity, currentVelocity, 0.02);

// 计算输出（位置闭环）
double output = controller.calculate(targetPosition, currentPosition, 0.02, false);

// 计算输出（完整 PIDSVA）
double output = controller.calculate(targetPosition, currentPosition, currentVelocity, currentAcceleration, 0.02);
```

## 多 Slot 配置示例

```
java
// 创建高速和低速配置
SlotConfig lowSpeedConfig = new SlotConfig()
    .withKP(0.2)
    .withKI(0.02)
    .withKD(0.1)
    .withKS(0.05)
    .withKV(0.1)
    .withKA(0.01);

SlotConfig highSpeedConfig = new SlotConfig()
    .withKP(0.1)
    .withKI(0.01)
    .withKD(0.05)
    .withKS(0.05)
    .withKV(0.1)
    .withKA(0.01);

// 创建控制器并设置两个 slot
PIDSVAController controller = new PIDSVAController()
    .withSlot(0, lowSpeedConfig)
    .withSlot(1, highSpeedConfig);

// 根据速度切换 slot
if (currentVelocity < 500) {
    controller.setSlot(0); // 使用低速配置
} else {
    controller.setSlot(1); // 使用高速配置
}

// 计算输出
double output = controller.calculate(targetVelocity, currentVelocity, 0.02);
```

## 注意事项

1. **参数调优**：PID 和 SVA 参数需要根据具体的电机和机械结构进行调优，建议通过实验确定最佳参数。

2. **积分限幅**：设置合理的积分上限可以防止积分饱和，提高系统稳定性。

3. **时间间隔**：`dt` 参数应与控制循环的实际时间间隔一致，通常为 0.02 秒（50Hz）。

4. **输出限幅**：设置合理的输出范围，防止电机过载。

5. **多 Slot 使用**：根据不同的控制场景（如不同速度范围）使用不同的参数配置，可以提高控制效果。

## 总结

PIDSVAControllers 包提供了灵活、强大的控制工具，可用于 FTC 机器人的各种控制场景。通过合理配置参数和选择合适的控制器类型，可以实现高精度、高稳定性的运动控制。