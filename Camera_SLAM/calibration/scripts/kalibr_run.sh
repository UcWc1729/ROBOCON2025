#!/bin/bash

# Kalibr 相机-IMU 联合标定执行脚本
# 将 ROS 2 bag 转换为 ROS 1 并运行 Kalibr 标定

set -e

# 颜色输出函数
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 检查输入参数
if [ $# -ne 1 ]; then
    print_error "Usage: $0 <bag_name>"
    print_error "Example: $0 calib_20241201_143000"
    exit 1
fi

BAG_NAME=$1
BAG_PATH="../data/${BAG_NAME}"

# 检查bag文件是否存在
if [ ! -d "$BAG_PATH" ]; then
    print_error "Bag directory not found: $BAG_PATH"
    exit 1
fi

print_info "=============================================="
print_info "Kalibr 相机-IMU 联合标定执行"
print_info "=============================================="
print_info "Input bag: $BAG_NAME"

# 检查Kalibr环境
print_step "1. Checking Kalibr environment..."
if ! command -v kalibr_calibrate_cameras &> /dev/null; then
    print_error "Kalibr not found! Please install Kalibr first."
    echo "Installation guide: https://github.com/ethz-asl/kalibr/wiki/installation"
    exit 1
fi

# 检查ROS1桥接工具
if ! command -v rosbags-convert &> /dev/null; then
    print_warn "rosbags-convert not found, installing..."
    pip3 install rosbags
fi

# 创建工作目录
WORK_DIR="../data/calibration_${BAG_NAME}"
mkdir -p "$WORK_DIR"
cd "$WORK_DIR"

print_step "2. Converting ROS2 bag to ROS1 format..."
# 转换bag格式 (ROS2 -> ROS1)
rosbags-convert "../${BAG_NAME}" --dst "${BAG_NAME}_ros1.bag"

if [ ! -f "${BAG_NAME}_ros1.bag" ]; then
    print_error "Failed to convert bag to ROS1 format!"
    exit 1
fi

print_info "✓ Bag conversion completed"

# 创建标定板配置文件
print_step "3. Creating calibration target configuration..."
cat > april_6x6_50x50cm.yaml << EOF
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088     # 标签边长 (米)，根据实际标定板调整
tagSpacing: 0.3    # 标签间距比例，根据实际标定板调整
codeOffset: 0
EOF

print_info "✓ Target configuration created: april_6x6_50x50cm.yaml"

# 创建IMU配置文件  
print_step "4. Creating IMU configuration..."
cat > imu.yaml << EOF
#Accelerometers
accelerometer_noise_density: 1.86e-03   #Noise density (continuous-time)
accelerometer_random_walk: 4.33e-04     #Bias random walk

#Gyroscopes  
gyroscope_noise_density: 1.87e-04       #Noise density (continuous-time)
gyroscope_random_walk: 2.66e-05         #Bias random walk

rostopic: /imu/data_raw                  #IMU话题名称
update_rate: 100.0                       #IMU频率
EOF

print_info "✓ IMU configuration created: imu.yaml"

# 执行相机内参标定
print_step "5. Running camera calibration..."
kalibr_calibrate_cameras \
    --bag "${BAG_NAME}_ros1.bag" \
    --topics /p100r/color/image_raw \
    --models pinhole-radtan \
    --target april_6x6_50x50cm.yaml \
    --bag-from-to 5 55

if [ $? -ne 0 ]; then
    print_error "Camera calibration failed!"
    exit 1
fi

print_info "✓ Camera calibration completed"

# 查找相机标定结果文件
CAMERA_YAML=$(ls -t *camchain*.yaml | head -n 1)
if [ ! -f "$CAMERA_YAML" ]; then
    print_error "Camera calibration result not found!"
    exit 1
fi

print_info "Camera calibration result: $CAMERA_YAML"

# 执行相机-IMU联合标定
print_step "6. Running camera-IMU calibration..."
kalibr_calibrate_imu_camera \
    --bag "${BAG_NAME}_ros1.bag" \
    --cam "$CAMERA_YAML" \
    --imu imu.yaml \
    --target april_6x6_50x50cm.yaml \
    --bag-from-to 5 55

if [ $? -ne 0 ]; then
    print_error "Camera-IMU calibration failed!"
    exit 1
fi

print_info "✓ Camera-IMU calibration completed"

# 查找最终标定结果
RESULT_YAML=$(ls -t *imu_cam*.yaml | head -n 1)
if [ ! -f "$RESULT_YAML" ]; then
    print_error "Final calibration result not found!"
    exit 1
fi

# 复制结果到文档目录
print_step "7. Organizing calibration results..."
mkdir -p "../../doc/calibration_results"
cp *.yaml ../../doc/calibration_results/
cp *.pdf ../../doc/calibration_results/ 2>/dev/null || true

print_info "✓ Results copied to ../doc/calibration_results/"

# 显示标定结果摘要
print_info "=============================================="
print_info "标定完成！结果文件："
print_info "=============================================="
echo "📄 相机内参: ../../doc/calibration_results/$(basename $CAMERA_YAML)"
echo "📄 相机-IMU外参: ../../doc/calibration_results/$(basename $RESULT_YAML)"
echo "📊 标定报告: ../../doc/calibration_results/*.pdf"
echo ""
print_info "Next steps:"
echo "1. 检查标定报告中的重投影误差和残差"
echo "2. 将外参结果更新到 tf_pub/launch/static_tf.launch.py"
echo "3. 将相机内参更新到相机驱动配置"
echo "4. 运行系统验证标定质量"

print_info "Calibration script completed successfully!" 