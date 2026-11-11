# 总线与消息

> sim 和 custom 是两个独立循环。它们互不认识，只通过总线和消息相遇。这篇文档说清这条通道。

## 一、一句话

**两条总线、两个通信模块、一套消息契约。**

- **`public_bus`** —— sim 和 custom 之间的**唯一**通道。两个方向都走它。
- **`sys_bus`** —— sim 内部通道，custom 不碰。
- **`SimComm` / `CustomComm`** —— 两侧各自的通信模块。它们管数据进出。
- **`core/bus/dataflow/`** —— 所有消息类型的定义。字段 + 语义约定都在那。

## 二、为什么是异步的

sim 和 custom 是**两个独立循环**：

```
sim 循环（200 Hz）                 custom 循环（300 Hz）
──────────────────                 ──────────────────────
物理步进                             keeper 每 1/300 s 叫 tick
传感器出检测                          pipeline 从 comm 取最新输入
电控驱动云台                         跑 pose_solver → tracker（火控在里边）
渲染                                发预测/控制回总线
```

**谁快谁慢都不影响对方。** sim 不等待 custom 处理完一帧；custom 也不等待 sim 出下一帧。

这靠**单槽缓存**实现：一帧检测从 sim 发出后，覆盖 `CustomComm` 里的"最新一帧"；custom 下次 tick 拿到的就是它——**永远是最新那一份**。

**为什么不是队列**：自瞄只关心最新帧。队列会累积延迟——用 100ms 前的检测去打现在的目标，是错的。单槽让慢的那侧自动丢帧，永远处理最新的。

## 三、数据流一图

```
┌─────────────────────── sim 循环 ────────────────────────┐
│                                                          │
│  sensor 出检测 ──→ collect_self_state()                  │
│       ↓                                                  │
│  SimComm.publish_frame(frame, state)                     │
│       ↓                                                  │
│  ┌──────────────── public_bus ────────────────────┐      │
│  │  topic: "detection"   payload: DetectionFrame  │      │
│  │  topic: "self_state"  payload: SelfState       │      │
│  └────────────────────────────────────────────────┘      │
│       ↑                                                  │
│  SimComm.latest_control()                                │
│       ↑                                                  │
│  control_manager.update(t, solution)                     │
│       ↑                                                  │
│  ┌──────────────── public_bus ────────────────────┐      │
│  │  topic: "control"     payload: ShotSolution    │      │
│  └────────────────────────────────────────────────┘      │
│                                                          │
└──────────────────────────────────────────────────────────┘
                          ↓ ↑
┌─────────────────────── custom 循环 ──────────────────────┐
│                                                          │
│  CustomComm 订阅 "detection" / "self_state"              │
│       ↓ 缓存最新一份                                     │
│  pipeline.tick()  ←── keeper 按 target_fps 调            │
│       ↓                                                  │
│  pose_solver.solve(frame, self_state) → Observation      │
│       ↓                                                  │
│  tracker.step(obs, self_state) → List[Prediction]        │
│       ↓                                                  │
│  publish("pred/robot" | "pred/outpost" | "pred/rune")    │
│       ↓                                                  │
│  tracker 内部发 ShotSolution                              │
│       ↓                                                  │
│  publish("control", solution)                            │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

## 四、两个通信模块

### 4.1 SimComm

```python
# sim/comm.py
class SimComm:
    def __init__(self):
        self._latest_control = None
        public_bus().subscribe(PublicMessages.CONTROL, self._on_control)

    def publish_frame(self, frame, self_state):
        """发一帧检测 + 同一刻的自车状态。"""
        public_bus().publish(PublicMessages.DETECTION, frame)
        public_bus().publish(PublicMessages.SELF_STATE, self_state)

    def latest_control(self):
        """取 custom 发来的最新控制指令（可能为 None）。"""
        return self._latest_control
```

**sim 侧调它**：

- `simulator._advance` 里，sensor 出帧后调 `publish_frame`
- `simulator._advance` 里，`control_manager.update` 前调 `latest_control`

**它只做两件事：发检测+状态，收控制。** 不碰 sys_bus、不认识 custom、不知道 pipeline 分几段。

### 4.2 CustomComm

```python
# custom/comm.py
class CustomComm:
    def __init__(self):
        self._latest_frame = None
        self._latest_state = None
        public_bus().subscribe(PublicMessages.DETECTION, self._on_frame)
        public_bus().subscribe(PublicMessages.SELF_STATE, self._on_state)

    def latest_inputs(self):
        """取最新一帧的 (检测, 自车状态)。"""
        return self._latest_frame, self._latest_state

    def publish_control(self, solution):
        """发控制指令。"""
        public_bus().publish(PublicMessages.CONTROL, solution)
```

**custom 侧调它**：

- `pipeline.tick` 里调 `latest_inputs` 拿输入
- `tracker` 内部直接 `public_bus().publish(CONTROL, solution)` 发控制——`publish_control` 只是同一件事的便捷写法，当前没人调它

**它只做两件事：收检测+状态，发控制。** 不认识 sim、不知道 sim 的内部结构。

### 4.3 单槽缓存为什么安全

写和读是不同线程，但**没有锁**。为什么？

- CPython 的 GIL 保证**单个变量赋值是原子的**——`self._latest_frame = frame` 不会让读者看到"半个对象"。
- frame 是每次新构造的**完整对象**，sim 不会原地改它。

**唯一的隐患**：读者可能拿到"新 frame + 旧 state"。这在两个独立变量覆盖时会短暂存在，但对自瞄影响可忽略——state 只用于位姿换算，一帧 20ms 的延迟对云台/相机姿态影响很小。

## 五、sys_bus

**custom 不碰 sys_bus。** 它是 sim 内部的通道：

| topic | 发布方 | 消费方 | payload |
|---|---|---|---|
| `fire` | ControlManager | BulletManager | dict（is_fire + reason 等诊断字段） |
| `reset` | Simulator | Keeper | None（通知 custom 复位） |

**它和 public_bus 的区别是"谁在用"**：

- **`sys_bus`** —— 只在系统侧（sim + entrance）内部用
- **`public_bus`** —— 系统侧和 custom 之间的唯一通道

**如果你的 custom 想订阅 `fire` 事件**——比如"开火后把当前目标锁住"——**做不到**。`fire` 在 `sys_bus` 上，custom 看不见。

**这是设计选择**：发弹的执行在 sim 侧，custom 只负责"推荐打哪、该不该打"。custom 不该知道"子弹到底出没出膛"。

## 六、消息类型

`core/bus/dataflow/` 里的消息是**跨层契约**。每个 dataclass 的 docstring 里写着字段的语义约定。

### 6.1 DetectionFrame

一帧检测。**基本上是 2D 图像点**——唯一的例外是圆靶（符）：传感器直接把靶心 3D 位置放进 `center`。

```python
@dataclass
class Detection:
    entity_type: EntityType      # 分类结果（Hero / Infantry / ... / Rune）
    image_points_2d: np.ndarray  # (4, 2) 像素四角点，顺序 LT / RT / RB / LB
    armor_size: str = ""          # "small" / "large"
    lit: bool = False             # 只有符用：这块灯臂是否点亮
    center: Optional[np.ndarray] = None  # 只有圆靶用：靶心 3D 位置（世界系）

@dataclass
class DetectionFrame:
    detections: List[Detection] = field(default_factory=list)
    timestamp: float = 0.0
```

**要点**：

- **空帧也发**（`detections=[]`）——采样到了但一块板都没看到，也发一帧，时间戳是新的。下游靠它知道"这一拍是空的"，而不是"沿用上一拍"。**与"没到 `sensor_fps` 采样点"不同**：那种情况 `get_obs` 返回 `None`，不发消息。
- **不含目标身份**——谁看到哪块板由 tracker 自己维护。
- **`center` 只有圆靶用**——圆靶没有板面方向可解，跳过 PnP，靶心直接给（世界系）。你的 `PoseSolver` 遇到 `entity_type is Rune` 要把它原样透传，别丢进 PnP。

### 6.2 SelfState

一帧自车状态。

```python
@dataclass
class SelfState:
    chassis: RigidState   # 底盘
    gimbal: RigidState    # 云台
    muzzle: RigidState    # 枪口
    camera: CameraState   # 相机（含内参 K）

@dataclass
class RigidState:
    pos: np.ndarray  # (3,) 世界系绝对坐标
    rpy: np.ndarray  # (3,) 世界系欧拉角
    vel: np.ndarray  # (3,)
    omg: np.ndarray  # (3,)

@dataclass
class CameraState:
    pos: np.ndarray  # (3,)
    rpy: np.ndarray  # (3,)
    K: np.ndarray    # (3,3) 内参
```

**要点**：

- **所有 pos 是世界系绝对坐标**——不是 G 系相对量。要 G 系相对量，自己减 `gimbal.pos`。
- `gimbal.rpy[1]` 是 pitch，**正值为低头**。
- `camera.K` 的主点是 **(0, 0)**，即图像中心。

### 6.3 Observation

一帧位姿观测。由 `PoseSolver` 产出，供 `Tracker` 消费。

```python
@dataclass
class ArmorObservation:
    entity_type: EntityType
    rel_pos: np.ndarray              # (3,) G 系或 W 系（看 cfg.sim.frame）
    rel_rpy: np.ndarray              # (3,) 绝对系
    armor_size: str = ""
    image_points_2d: Optional[np.ndarray] = None
    lit: bool = False

@dataclass
class Observation:
    obs_armors: List[ArmorObservation] = field(default_factory=list)
    timestamp: float = 0.0
```

**要点**：

- **`rel_pos` 是板的位置，不是车的中心**。一块车上几块板就有几个 `ArmorObservation`。
- **`rel_rpy[2]` 是板在绝对系下的偏航**——不是相对相机的角度。
- **消息里没有目标身份**——同一帧看到的两块板，是不是同一辆车的，由 tracker 自己判断。
- **空帧也发**（`obs_armors=[]`）——只要这一 tick 有输入（DetectionFrame + SelfState），pipeline 就发一条 Observation，即使一块板都没解出来。检测端和解算端在这点上一致：都是"这一拍看了，结果为空"，而不是让下游沿用上一拍。真正的不发只有一种：没到 `sensor_fps` 采样点。

### 6.4 Prediction

一帧预测。由 `Tracker` 产出，**供渲染画点**。

```python
@dataclass
class Prediction:
    target_type: TargetType          # ROBOT / OUTPOST / RUNE
    center: np.ndarray               # (3,) 目标中心
    armors: List[np.ndarray]         # 各装甲板位置，第一条是"该打的"
    timestamp: float                 # 必须等于输入的 obs.timestamp
    is_tracking: bool
    aim_rate: Optional[np.ndarray] = None
    gimbal_target: Optional[GimbalTarget] = None  # 扩展位：当前没有消费方
```

**要点**：

- **`armors[0]` 是"未来时刻"的位置**——不是当前板位置。Tracker 已按 `total_delay` 前推过。
- **`timestamp` 必须与输入一致**——它是配对键；目前没有代码强制检查，但你自己做前推、做延迟补偿时全靠它。
- **一个 tracker 一帧可以发多条**——按 `target_type` 分通道。
- **火控信息不在本条里**——`fire` / `fire_mode` / `fire_key` 走 `ShotSolution`。
- **fps 不在这条里**——它是 pipeline 层的度量，走 `PIPELINE_STATS`。

### 6.5 ShotSolution

自瞄发出的**火控指令**。由 `Tracker` 内部产出，**驱动云台并决定是否发弹**。

```python
@dataclass
class ShotSolution:
    yaw: float           # 绝对系下的云台 yaw
    pitch: float         # 抬头为正（云台下发时要取负）
    yaw_rate: float = 0.0    # 瞄点视线的角速度，给电控做前馈
    pitch_rate: float = 0.0
    fire: bool = False       # 自瞄的推荐："这一帧该不该扣扳机"
    fire_mode: str = "auto"  # "auto"（连发）/ "single"（单发）
    fire_key: Optional[int] = None  # 自瞄估计的"当前瞄的板/灯臂编号"
    cooldown: float = 0.0    # 自瞄建议的最小两发间隔；0 = 用电控默认
    timestamp: float = 0.0
```

**要点**：

- **`pitch` 抬头为正**——`ControlManager` 下发时会取负。
- **`yaw_rate` / `pitch_rate` 是前馈**——没有前馈，云台 PID 必然滞后于瞄点轨迹。
- **`fire` 是"推荐"，不是"裁决"**——它说的是"自瞄认为该打了"（板朝向 / 灯臂点亮 / 板接近虚拟点），**不是"云台到位了"**。电控拿到 `fire=True` 后还要过自己的关：用户开关（KP9 / KP8）、单发判据、冷却。
- **`cooldown` 是建议值**——留 0 就用电控默认（`cfg.sim.aim.cooldown`，0.1s）；符要给更大的值（比如 0.8s）。
- **`fire_mode`**：
    - `"auto"`（连发）：`fire=True` 就打，电控按冷却节流。
    - `"single"`（单发）：电控按 `fire_key` 变化或 `fire` 上升沿判"新一发"。
- **`fire_key`** 是自瞄对"当前瞄哪块板/灯臂"的**估计**——车是装甲板 id，符是灯臂 id，虚拟点模式为 `None`。**不是上帝视角**：它是自瞄从观测角度推出来的，连续帧上同一块板 id 稳定；同一辆车从不同角度看可能估出不同编号，但那和"仿真器告诉它这是几号"是两回事。
- **`timestamp` 必须与产生它的那条 `Prediction` 一致**。

### 6.6 PipelineStats

custom 循环的运行时度量。由 `Pipeline` 产出，供渲染层显示。

```python
@dataclass
class PipelineStats:
    fps: float = 0.0
```

**要点**：

- **它是 pipeline 层的度量，不是 tracker 的**——所以走独立的 topic。
- 发到 `public_bus` 的 `pipeline/stats` 通道，渲染层的 `BusCache` 订阅它。

## 七、Message 对象

**topic 和 payload 类型绑在一个对象里。**

```python
# core/bus/messages.py
@dataclass(frozen=True)
class Message:
    topic: str
    dataflow: Optional[Type] = None      # 承载的消息类型

class PublicMessages:
    DETECTION = Message("detection", DetectionFrame)
    SELF_STATE = Message("self_state", SelfState)
    OBS = Message("obs", Observation)
    CONTROL = Message("control", ShotSolution)
    DEBUG_TEXT = Message("debug/text", DrawText)
    DEBUG_PLOT = Message("debug/plot", dict)
    PIPELINE_STATS = Message("pipeline/stats", PipelineStats)

    @staticmethod #直接退化成普通函数,这样好在内聚,好看
    def pred(target_type) -> Message:
        return Message(f"pred/{target_type.value}", Prediction)
```

**`DEBUG_PLOT` 的字段是曲线面板认的固定键名**（`obs_yaw` / `obs_pitch` / `pred_yaw` / `pred_pitch`，单位 rad），发法见 `docs/writing_your_tracker.md` §五。

**用途**：

- 消费点读 `PublicMessages.DETECTION.dataflow` 就知道 payload 是什么类型。
- IDE 能补全 `Message.topic` 和 `Message.dataflow`。

**但 EventBus 不强制检查类型**。`publish(topic, data)` 里的 `data` 是 `Any`——消息契约是**约定**，不是**运行时检查**。

**custom 也可以用裸字符串作 topic**：

```python
public_bus().publish("my_custom_topic", my_data)
```

EventBus 会把它当成普通 topic。**但不建议**——标准 topic 有文档、有类型、别人看得懂；裸字符串是你自己的私有约定。

## 八、时序

一帧内的顺序：

```
1. sensor_manager.get_obs() 产出 DetectionFrame
2. simulator.collect_self_state() 产出 SelfState
3. SimComm.publish_frame(frame, state)
       ↓ [public_bus]
4. CustomComm._on_frame / _on_state 缓存最新
       ↓
5. keeper 叫 pipeline.tick()（按 target_fps，与 sim 帧率无关）
6. pipeline:
       pose_solver.solve(frame, state) → Observation
       publish(OBS)  ← 渲染订阅
       tracker.step(obs, state) → List[Prediction]
       publish(pred/<type>)  ← 渲染订阅
       tracker 内部发 ShotSolution
       publish(CONTROL)  ← SimComm 订阅
       ↓ [public_bus]
7. SimComm._on_control 缓存最新
       ↓
8. simulator 下一帧 control_manager.update(t, solution) 读它
```

**关键点**：

- **两条消息背靠背发、中间没有等待**：先 `DETECTION`、后 `SELF_STATE`。custom 的 tick 恰好落在两条之间时，会拿到"新 frame + 旧 state"（§4.3 说过，对自瞄影响可忽略）。
- **一帧最多发一次**——`pipeline.tick` 里没有输入就什么都不做。
- **`CONTROL` 到达 sim 侧可能滞后一帧**——`simulator._advance` 里先读 `latest_control`，再产生检测。这一帧读的是上一帧 custom 发的控制。

## 九、写 Tracker 时要记住的

- **输入是 `Observation`**，不是 `DetectionFrame`——PnP 在你自己的 `PoseSolver` 里做完了。
- **输出是 `List[Prediction]`**。第一条是"该打谁"。
- **`armors[0]` 是未来时刻的位置**。你要自己前推。
- **`timestamp` 必须和输入一致**。这条最容易忘；它现在没有代码检查，但它是你后面前推的时间基准。
- **坐标是 G 系还是 W 系看 `cfg.sim.frame`**。G 系下不用减 `gimbal.pos`。
- **火控走 `ShotSolution`**——不是 `Prediction`。要在 tracker 内部算出 `ShotSolution` 并发到 `CONTROL` 通道。

## 十、常见问题

### 我发的 Prediction 到了 sim 但云台不动？

**Prediction 不驱动云台。** 云台看的是 `ShotSolution`——它在 `CONTROL` 通道上。

检查：

1. 你的 tracker 里有没有发 `ShotSolution`（`public_bus().publish(PublicMessages.CONTROL, solution)`）？
2. `solution.timestamp` 和这一帧的 `obs.timestamp` 一致吗？
3. 电控判了什么？拒绝原因只有这几种：`no_solution` / `auto_aiming_off` / `auto_fire_off` / `aim_off` / `same_target` / `not_rising_edge` / `cooldown`。

**但 `reason` 发在 `sys_bus` 的 `fire` 事件上，custom 看不到，信息面板也不显示**——想确认就打开 `sim/managers/control_manager.py` 对着看，或者在自己的 `step()` 里 `debug_text(...)` 把当时的判断打出来。

### 我发了两条 Prediction 但 sim 只认一条？

**不是"认一条"。** `Prediction` 现在只给渲染——`ControlManager` 不订阅它。你要发几条都可以，渲染按 `target_type` 各画各的。

### 为什么 `Prediction.armors[0]` 是未来时刻的？

因为它是**瞄点的前推结果**。子弹飞过去要时间——你按 `total_delay` 前推后的位置就是瞄点。渲染画它，看到的也是"该往哪打"。

### 我能不能不用 `public_bus`，直接调 sim 的函数？

**不能。** `sim` 与 `custom` 互相不认识。你想让 sim 做事，只能通过 `public_bus` 发消息。

### 我的 `fire=True` 但子弹不出膛？

按顺序检查：

1. **`ControlManager` 的用户开关**：KP9（`auto_aiming`）、KP8（`auto_fire`）都开了吗？
2. **冷却**：距上一发超过 `cooldown`（默认 0.1s，或你在 `ShotSolution.cooldown` 里给的值）了吗？
3. **单发判据**：`fire_mode="single"` 时——如果 `fire_key` 非 None（符），看 `fire_key` 是否变；如果为 None（车虚拟点），看 `fire` 是否 0→1。
4. **`BulletManager` 还要过 `fire_delay`（0.02s）** —— 但这一层很少出错。

## 十一、一句话

**两条总线：`sys_bus`（sim 内部）+ `public_bus`（跨边界）。**

**两个通信模块：`SimComm` + `CustomComm`。各自负责一侧的进与出。**

**单槽缓存，不是队列——只留最新。**

**你写 tracker 时只需要碰 `public_bus` 上的消息：`detection` / `self_state` / `obs` / `pred/<type>` / `control`。**