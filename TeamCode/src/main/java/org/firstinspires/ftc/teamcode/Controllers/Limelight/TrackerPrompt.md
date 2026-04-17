 `Tracker.java` 代码**已完整实现**所描述的全部功能，核心逻辑如下：


### 1. 目标聚类与分组  
- **聚类算法**：`clusterDetections()` 通过 BFS 将距离小于 `distanceThreshold` 的检测归为一组。单个孤立的检测也会形成大小为 1 的组。  
- **目标表示**：内部类 `Target` 持有组成员（`Member`）及其坐标均值。`CandidateTarget` 作为未确认目标的暂存结构。

---

### 2. 坐标计算  
- `Target.updateCenter()` 仅使用**连续出现帧数 ≥ confirmationFrames** 的成员坐标计算平均值，确保坐标稳定。  
- 中心坐标会随成员增删自动更新。

---

### 3. 延迟确认与移除  
| 场景 | 实现机制 |
|------|----------|
| **新球出现** | 新建 `CandidateTarget`，连续出现 ≥ `confirmationFrames` 帧后转为 `Target`。 |
| **已有球消失** | 成员 `consecutiveMisses` 累加，达到 `removalFrames` 后从所属 `Target` 中移除。 |
| **Target 内所有球消失** | `removalPending` 计数累加，达到 `removalFrames` 且无活跃成员时删除该 `Target`。 |
| **检测框短暂抖动** | 不立即添加/删除，完全满足滞后滤波要求。 |

---

### 4. 最优目标选择  
`getBestTarget()` 计算得分公式：  
```
score = activeMemberCount / distanceToCamera
```
- `activeMemberCount`：该目标内已确认的成员数量。  
- `distanceToCamera`：目标中心到摄像头（原点）的欧氏距离。  
- 返回**得分最高**的 `Target` 对象，其 `centerX`、`centerY` 即为相对坐标。

---

### 5. 其他细节验证  
- `targets` 用 `List<Target>` 动态维护，支持增删改查。  
- `Detector` 与 `Projector` 配合完成检测到世界坐标的转换，满足 FTC 应用场景。  
- 多线程安全虽未显式声明，但 `update()` 在单线程循环调用中工作正常。
