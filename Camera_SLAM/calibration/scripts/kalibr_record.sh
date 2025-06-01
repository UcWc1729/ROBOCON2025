#!/bin/bash

# Kalibr 相机-IMU 联合标定数据录制脚本
# 用于录制标定板序列和IMU数据，供后续离线标定使用

set -e

# 配置参数
RECORD_TIME=60  # 录制时间(秒)
BAG_NAME="calib_$(date +%Y%m%d_%H%M%S)"
TOPICS="/p100r/color/image_raw /p100r/depth/image_raw /p100r/color/camera_info /imu/data_raw"

# 颜色输出函数
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS environment not sourced!"
    exit 1
fi

print_info "ROS 2 $ROS_DISTRO environment detected"

# 检查必要的话题是否存在
print_info "Checking required topics..."
for topic in $TOPICS; do
    if ! ros2 topic list | grep -q "^$topic$"; then
        print_warn "Topic $topic not found, make sure all sensors are running"
    else
        print_info "✓ Found topic: $topic"
    fi
done

# 显示标定须知
print_info "=============================================="
print_info "Kalibr 相机-IMU 联合标定数据录制"
print_info "=============================================="
echo ""
print_warn "标定前准备："
echo "1. 确保相机和IMU驱动已启动"
echo "2. 准备好标定板 (建议使用 6x6 Aprilgrid)"
echo "3. 标定环境光照充足，避免运动模糊"
echo "4. 准备进行多角度、多方向的激励运动"
echo ""
print_warn "录制期间要求："
echo "1. 标定板在相机视野内保持可见"
echo "2. 进行平移、旋转、俯仰等多种运动"
echo "3. 运动要平滑，避免急停急转"
echo "4. 覆盖相机视野的各个区域"
echo "5. 包含足够的IMU激励 (加速度和角速度变化)"
echo ""

read -p "Press Enter to continue..."

# 创建输出目录
mkdir -p ../data
cd ../data

print_info "Starting bag recording for ${RECORD_TIME} seconds..."
print_info "Output bag: ${BAG_NAME}.bag"
print_info "Topics: $TOPICS"

# 开始录制
ros2 bag record -a -o $BAG_NAME --max-bag-duration $RECORD_TIME &
RECORD_PID=$!

# 显示倒计时
for ((i=$RECORD_TIME; i>0; i--)); do
    printf "\rRecording... %02d:%02d remaining" $((i/60)) $((i%60))
    sleep 1
done

# 停止录制
kill $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true

print_info ""
print_info "Recording completed!"

# 检查bag文件
if [ -d "${BAG_NAME}" ]; then
    print_info "Bag file created: ${BAG_NAME}"
    
    # 显示bag信息
    print_info "Bag info:"
    ros2 bag info ${BAG_NAME}
    
    print_info ""
    print_info "Next steps:"
    echo "1. Run 'bash kalibr_run.sh ${BAG_NAME}' to start calibration"
    echo "2. Make sure you have the calibration target YAML file ready"
    echo "3. Check ../doc/ for calibration results when complete"
    
else
    print_error "Failed to create bag file!"
    exit 1
fi

print_info "Script completed successfully!" 