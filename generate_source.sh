#!/bin/bash

# 현재 스크립트 위치로 이동 (루트에서 실행되도록)
cd "$(dirname "$0")" || exit

OUTPUT_FILE="sources.cmake"

# 새 sources.cmake 파일 시작
echo "# Auto-generated sources list" > "$OUTPUT_FILE"
echo "set(SOURCE_FILES" >> "$OUTPUT_FILE"

# bt_runner.cpp 명시적으로 추가
if [ -f src/bt_runner.cpp ]; then
  echo "  src/bt_runner.cpp" >> "$OUTPUT_FILE"
fi

# bt_nodes 디렉토리의 .cpp 파일 추가
if [ -d src/bt_nodes ]; then
  find src/bt_nodes -name "*.cpp" | sort | while read -r file; do
    echo "  $file" >> "$OUTPUT_FILE"
  done
fi

# utils 디렉토리의 .cpp 파일도 추가 (있을 경우)
if [ -d src/utils ]; then
  find src/utils -name "*.cpp" | sort | while read -r file; do
    echo "  $file" >> "$OUTPUT_FILE"
  done
fi

echo ")" >> "$OUTPUT_FILE"

# 완료 메시지
echo "✅ Generated $OUTPUT_FILE"

echo ""
echo "📌 [CMakeLists.txt 사용 예시]"
echo ""
echo "include(\${CMAKE_CURRENT_SOURCE_DIR}/sources.cmake)"
