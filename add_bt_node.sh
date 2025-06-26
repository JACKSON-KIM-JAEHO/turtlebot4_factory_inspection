#!/bin/bash

NODE_NAME=$1

if [ -z "$NODE_NAME" ]; then
  echo "❌ 노드 이름을 입력하세요. 예: ./add_bt_node.sh MoveToTarget"
  exit 1
fi

HEADER_DIR="include/turtlebot4_factory_inspection"
CPP_DIR="bt_nodes"
HEADER_FILE="${HEADER_DIR}/${NODE_NAME}.hpp"
CPP_FILE="${CPP_DIR}/${NODE_NAME}.cpp"

# 헤더가드용 상수 변환 (CamelCase → 대문자_언더스코어)
GUARD_NAME=$(echo "TURTLEBOT4_FACTORY_INSPECTION_${NODE_NAME}_HPP_" | sed 's/\([a-z]\)\([A-Z]\)/\1_\2/g' | tr '[:lower:]' '[:upper:]')

# 디렉토리 생성
mkdir -p "$HEADER_DIR" "$CPP_DIR"

# 이미 존재하는 경우 개별 안내
if [ -e "$HEADER_FILE" ]; then
  echo "⚠️  헤더 파일이 이미 존재합니다: $HEADER_FILE"
fi

if [ -e "$CPP_FILE" ]; then
  echo "⚠️  소스 파일이 이미 존재합니다: $CPP_FILE"
fi

if [ -e "$HEADER_FILE" ] || [ -e "$CPP_FILE" ]; then
  echo "⛔ 생성이 중단되었습니다. 파일을 삭제하거나 다른 노드 이름을 입력하세요."
  exit 1
fi

# 헤더 파일 생성
cat <<EOF > "$HEADER_FILE"
#ifndef ${GUARD_NAME}
#define ${GUARD_NAME}

#include <behaviortree_cpp_v3/action_node.h>

namespace turtlebot4_factory_inspection {

class ${NODE_NAME} : public BT::SyncActionNode
{
public:
  ${NODE_NAME}(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace turtlebot4_factory_inspection

#endif  // ${GUARD_NAME}
EOF

# 소스 파일 생성
cat <<EOF > "$CPP_FILE"
#include "turtlebot4_factory_inspection/${NODE_NAME}.hpp"

namespace turtlebot4_factory_inspection {

${NODE_NAME}::${NODE_NAME}(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList ${NODE_NAME}::providedPorts()
{
  return {};
}

BT::NodeStatus ${NODE_NAME}::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
EOF

echo "✅ BT 노드 생성 완료: ${NODE_NAME}"
echo " - 헤더: $HEADER_FILE"
echo " - 소스: $CPP_FILE"

# sources.cmake 자동 갱신
if [ -f ./generate_sources.sh ]; then
  ./generate_sources.sh > /dev/null
  echo "📄 sources.cmake 자동 갱신 완료"
else
  echo "ℹ️  generate_sources.sh 파일이 없어 sources.cmake는 갱신되지 않았습니다."
fi
