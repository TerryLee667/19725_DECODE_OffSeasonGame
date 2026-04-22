# AutoSelect 使用说明

## 概述
AutoSelect 是RK4跑打算法的自动选择器，根据目标位置和机器人状态自动计算最优的发射参数（仰角、初速度、旋转角）。

## 构造函数

### 默认构造函数
```java
AutoSelect autoSelect = new AutoSelect();
```
使用默认参数：
- 最大仰角：65度
- 初速度范围：0.0 - 10.0 m/s
- 仰角范围：45 - 65度

### 自定义参数构造函数
```java
ProjectileParameters params = new ProjectileParameters();
params.k = 0.0005;  // 阻力系数
params.n = 2.0;     // 阻力指数
params.v0 = 10.0;   // 默认初速度
params.deltaH = 1.0; // 高度差（炮口与目标）

AutoSelect autoSelect = new AutoSelect(params);
```

## 主要方法

### Select - 合并模式（推荐）
```java
AutoSelectResult result = autoSelect.Select(
    relativeX,    // 目标相对X坐标（m）
    relativeY,    // 目标相对Y坐标（m）
    robotVx,      // 机器人X方向速度（m/s）
    robotVy,      // 机器人Y方向速度（m/s）
    initialV0,    // 初始初速度（m/s）
    initialTheta  // 初始仰角（弧度）
);
```

**功能说明：**
- 同时输入初始初速度和仰角值
- 使用二分法算法同时优化初速度和仰角
- 输出最优的发射参数组合

## 返回结果

### AutoSelectResult
```java
public class AutoSelectResult {
    public final double theta;      // 仰角（弧度）
    public final double v0;         // 初速度（m/s）
    public final double turretPhi;  // 旋转角（弧度，与小车x轴夹角，逆时针为正）
    public final boolean success;   // 是否成功
    public final String message;    // 消息
    
    public double getThetaDegrees();      // 获取仰角（度）
    public double getTurretPhiDegrees(); // 获取旋转角（度）
}
```

## 使用示例

### 基本使用
```java
// 创建AutoSelect实例
AutoSelect autoSelect = new AutoSelect();

// 设置目标位置和机器人速度
double targetX = 4.5;  // 目标X坐标（m）
double targetY = 0.3;  // 目标Y坐标（m）
double robotVx = -0.5; // 机器人X速度（m/s）
double robotVy = 0.2;  // 机器人Y速度（m/s）

// 使用合并模式（同时输入初始初速度和仰角）
double initialV0 = 8.0;     // 初始初速度（m/s）
double initialTheta = Math.toRadians(55); // 初始仰角（55度）
AutoSelectResult result = autoSelect.Select(targetX, targetY, robotVx, robotVy, initialV0, initialTheta);

if (result.success) {
    System.out.printf("仰角: %.2f 度\n", result.getThetaDegrees());
    System.out.printf("初速度: %.2f m/s\n", result.v0);
    System.out.printf("旋转角: %.2f 度\n", result.getTurretPhiDegrees());
} else {
    System.out.println("求解失败: " + result.message);
}
```

## 配置方法

### 设置最优初速度列表
```java
List<Double> optimalV0List = new ArrayList<>();
optimalV0List.add(7.0);
optimalV0List.add(8.0);
optimalV0List.add(9.0);
autoSelect.setOptimalV0List(optimalV0List);
```

### 设置最优仰角列表
```java
List<Double> optimalThetaList = new ArrayList<>();
optimalThetaList.add(Math.toRadians(45));
optimalThetaList.add(Math.toRadians(55));
optimalThetaList.add(Math.toRadians(65));
autoSelect.setOptimalThetaList(optimalThetaList);
```

### 设置初速度范围
```java
autoSelect.setV0Range(0.0, 10.0); // 最小值，最大值
```

### 设置仰角范围
```java
autoSelect.setThetaRange(Math.toRadians(45), Math.toRadians(65)); // 最小值，最大值（弧度）
```

### 设置高度差
```java
autoSelect.setDeltaH(1.0); // 炮口与目标的高度差（m）
```

## 注意事项

1. **坐标系**：
   - X轴：机器人前进方向
   - Y轴：机器人左侧方向
   - 旋转角：与小车x轴夹角，范围-pi~pi，逆时针为正

2. **单位**：
   - 距离：米（m）
   - 速度：米/秒（m/s）
   - 角度：弧度（rad），显示时转换为度数

3. **参数范围**：
   - 初速度范围：0.0 - 10.0 m/s
   - 仰角范围：45 - 65度
   - 超出范围的参数将被拒绝

4. **求解失败处理**：
   - 检查目标位置是否在有效范围内
   - 调整初始初速度或仰角值
   - 尝试使用不同的初始参数组合
