# second_car 电机串口通信说明

STM32F103C8T6（HAL + CMake + VS Code）通过串口和“四路直流电机驱动板”通信，
实现 **正反转 + 调速 + 读取编码器**。

```
        ┌─────────────┐   USART1 (PA9/PA10) 115200   ┌──────────┐
        │  STM32F103  │ ───────────────────────────► │   电脑   │
        │ second_car  │ ◄─────────────────────────── │ 串口助手 │
        └──────┬──────┘       命令 / 调试信息          └──────────┘
               │ USART2 (PA2/PA3) 115200
               ▼
        ┌──────────────────┐
        │  四路电机驱动板   │  M1~M4 接电机 + 编码器
        │ （内部跑速度PID） │
        └──────────────────┘
```

## 1. 协议长什么样

全部是 **可见 ASCII 字符**，一帧 = `$` + 命令名 + `:` + 参数 + `#`，
参数之间用逗号分隔。所以用串口助手就能直接手动发命令。

```
下发：  $ spd : 300,-300,0,0 #
回传：  $ MSPD : 120.50,120.00,0.00,0.00 #
```

- `$`：帧头
- `#`：帧尾（收到 `#` 才算一帧完整数据）
- 数值：正数正转、负数反转

## 2. 下发给驱动板的命令

| 命令 | 例子 | 说明 |
|------|------|------|
| `$spd:a,b,c,d#` | `$spd:300,-300,0,0#` | **闭环速度**控制，M1~M4，范围 -1000~1000，正负决定转向 |
| `$pwm:a,b,c,d#` | `$pwm:1200,1200,0,0#` | 开环 PWM，范围 -3600~3600（不经过 PID） |
| `$upload:x,y,z#` | `$upload:0,1,1#` | 回传开关：x=总编码器 y=10ms增量 z=速度 |
| `$mtype:x#` | `$mtype:2#` | 电机型号 1:520 2:310 3:TT(带编码器) 4:TT |
| `$mline:x#` | `$mline:13#` | 编码器线数（查电机参数表） |
| `$mphase:x#` | `$mphase:30#` | 减速比（查电机参数表） |
| `$wdiameter:x#` | `$wdiameter:65#` | 轮子直径 mm |
| `$deadzone:x#` | `$deadzone:1600#` | PWM 死区 0~3600，用来消掉最小速度时的抖动 |
| `$MPID:p,i,d#` | `$MPID:0.5,0.01,0.1#` | 驱动板内部速度环 PID（改一次芯片会重启） |
| `$flash_reset#` | — | 恢复出厂 |
| `$read_flash#` | — | 查询配置 |
| `$read_vol#` | — | 查询电池电压 |

> `$MPID` 在有些固件版本里是小写 `$mpid:`。
> 如果改 PID 没反应，把 `Core/Src/motor_proto.c` 里的 `#define CMD_PID` 改成小写即可。

## 3. 驱动板回传的数据

| 帧 | 含义 |
|----|------|
| `$MAll:1,2,3,4#` | 四路**累计**编码器数（int32，可用于里程计） |
| `$MTEP:1,2,3,4#` | 四路 **10ms 内**的编码器增量（测速用） |
| `$MSPD:120.5,...#` | 四路速度，单位 **mm/s**（float） |

打开哪个用 `$upload:` 的第 1/2/3 个参数控制。
默认（`motor_proto_init()` 里）开的是 **10ms 增量 + 速度**。

## 4. 代码结构（4 个文件，各管一件事）

| 文件 | 作用 |
|------|------|
| `Core/Src/uart_io.c` | 两路串口收发：中断收字节 → 环形缓冲区；`printf` 重定向到 USART1 |
| `Core/Src/motor_proto.c` | 协议层：拼 `$spd:..#` 帧发出去；把 `$MSPD:..#` 解析成 `g_motor` |
| `Core/Src/pc_cmd.c` | 电脑端命令：把 `spd 300` 这种人类命令翻译成协议帧，并定时打印编码器 |
| `Core/Src/main.c` | 主循环只调用三个 `poll`，不做别的 |

主循环就是这么简单：

```c
while (1)
{
    motor_proto_poll(); /* 收驱动板回传的编码器数据 */
    pc_cmd_poll();      /* 收电脑命令并转成协议帧 */
    pc_report_poll();   /* 每 100ms 打印一次编码器数据 */
}
```

## 5. 电脑上怎么用

串口助手（115200 8N1，接 USART1）里输入命令 + 回车：

```
spd 300            四路都正转 300
spd -300           四路都反转 300
spd 300,-300,0,0   分别设置 M1~M4
pwm 1200           开环 PWM，用来试死区
stop               停车
upload 1,1,1       同时回传总编码器/10ms增量/速度
get                立刻打印一次编码器
mtype 2            电机型号
mline 13           编码器线数
mphase 30          减速比
deadzone 1600      死区
wdiameter 65       轮径
pid 0.5,0.01,0.1   速度环 PID
raw $read_vol#     直接发一帧
help               命令表
```

正常时会看到这样的回传打印（每 100ms 一行）：

```
[1234 ms] spd(mm/s): 120.50 -120.50 0.00 0.00 | d: 12 -12 0 0 | tot: 1350 1348 0 0
```

## 6. 第一次上车要做的配置

驱动板的参数存在它自己的 flash 里，配一次就行（顺序参考厂家例程）：

```
mtype 2          ← 你的电机型号
mphase 30        ← 减速比（查参数表）
mline 13         ← 编码器线数（查参数表）
wdiameter 65     ← 轮径，量出来的 mm
deadzone 1600    ← 死区，实测：从 pwm 1200 慢慢加到轮子能转
```

配置项每改一次驱动板会重启一次，是正常的。

## 7. VS Code 里怎么编译和下载

- 编译：`Ctrl+Shift+B`（默认任务 = “2. 编译 (Debug)”），
  或终端里 `cmake --preset Debug` + `cmake --build --preset Debug`
- 下载：任务 “3. 下载到芯片 (OpenOCD + ST-Link)”，
  用 DAPLink / J-Link 就改 `.vscode/tasks.json` 里的 `interface/*.cfg`
- 在线调试：装 Cortex-Debug 扩展，按 F5（`.vscode/launch.json` 已配好）

## 8. 几个容易踩的坑

1. **USART2 和 USART1 别接反**：USART1(PA9/PA10) 接电脑，USART2(PA2/PA3) 接驱动板。
   TX 接对方 RX。
2. **共地**：STM32、驱动板、电脑（USB 转串口）必须共地。
3. **USART2 中断**：CubeMX 里没勾选 `USART2 global interrupt`，
   所以 `uart_io.c` 里手动开了 NVIC 并写了 `USART2_IRQHandler`。
   如果你以后在 CubeMX 里勾上了，CubeMX 会生成同名函数，
   把 `uart_io.c` 里那份删掉即可。
4. **printf 里不能用 `%f`**：工程用的是 newlib-nano，浮点 printf 默认没开。
   代码里用 `motor_format_fixed()` 转成字符串打印。
5. **速度范围**：`$spd` 是 ±1000，`$pwm` 是 ±3600，超出范围驱动板直接忽略（这里代码已经帮你钳位）。
6. **VS Code 输出里那两行**
   `Unable to get build directory for 'second_car'` / `Cannot find project build information yet`
   只是 STM32 扩展在问 CMake Tools “build 目录在哪”，第一次打开工程、还没配置过时必然出现，
   `[Info]` 级别，**不影响编译和下载**。想让它消失：
   `Ctrl+Shift+P` → `CMake: Select Configure Preset` 选 `Debug` → `CMake: Configure`。
   IntelliSense 这边已经改成直接读 `build/Debug/compile_commands.json`（和 first_car 一样），
   编译过一次后代码跳转索引就正常了。

## 9. 附带的主机单元测试（可选）

`tests/host_test.c` 可以在电脑上直接验证“帧生成 + 帧解析”，不需要板子：

```bash
gcc -std=c11 -I Core/Inc -I tests/inc tests/host_test.c Core/Src/motor_proto.c Core/Src/pc_cmd.c -o test.exe
./test.exe
```
