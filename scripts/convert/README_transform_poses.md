# 位姿数据变换脚本使用说明

## 功能描述

`transform_poses.py` 脚本用于处理CSV格式的位姿数据文件，执行以下变换操作：

1. **位置变换**：
   - X轴数据和Y轴数据交换
   - 交换后的X轴数据取反
   - Z轴数据取反
   - 即：(x, y, z) → (-y, x, -z)

2. **方向变换**：
   - **保持原始四元数不变**，不进行任何旋转操作

## 使用方法

### 方法1：使用默认文件
```bash
cd /home/john/rosbag_evo/src/rosbag_contral
python3 scripts/convert/transform_poses.py
```
这将自动处理 `config/poses_lidar2body.csv` 文件并覆盖原文件。

### 方法2：指定输入文件
```bash
python3 scripts/convert/transform_poses.py path/to/your/poses.csv
```
这将处理指定的CSV文件并覆盖原文件。

### 方法3：指定输入和输出文件
```bash
python3 scripts/convert/transform_poses.py input.csv -o output.csv
```
这将处理 `input.csv` 文件并将结果保存到 `output.csv`。

## CSV文件格式要求

输入的CSV文件必须包含以下列：
- `x`：X轴位置
- `y`：Y轴位置
- `z`：Z轴位置
- `qx`：四元数X分量
- `qy`：四元数Y分量
- `qz`：四元数Z分量
- `qw`：四元数W分量

其他列（如 `index`, `timestamp`）会被保留但不会被修改。

## 变换公式

### 位置变换
```
新X = -原Y
新Y = 原X
新Z = -原Z
```

### 方向变换
四元数保持原始值不变：
```
qx_new = qx_original
qy_new = qy_original  
qz_new = qz_original
qw_new = qw_original
```

## 依赖包

- pandas：用于CSV文件读写
- numpy：用于数值计算
- scipy：用于四元数和旋转运算

## 示例

变换前的数据：
```
x=0.145576, y=-0.323060, z=0.380333
qx=-0.001788, qy=0.006879, qz=0.518897, qw=-0.854807
```

变换后的数据：
```
x=0.323060, y=0.145576, z=-0.380333
qx=-0.001788, qy=0.006879, qz=0.518897, qw=-0.854807
```

## 注意事项

- 脚本会显示变换前后的对比数据以供验证
- 默认情况下会覆盖原文件，建议先备份重要数据  
- 四元数保持原始值不变，不进行任何旋转变换
- 只有位置坐标会按照指定公式进行变换