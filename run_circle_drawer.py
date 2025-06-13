#!/usr/bin/env python3
"""
원 그리기 GUI 실행 스크립트
"""

from GUI import CircleDrawerGUI

if __name__ == "__main__":
    # DUAL MODE로 실행: 시뮬레이션 화면 + 실제 로봇 동시 구동
    # 실제 로봇이 연결되지 않은 경우 자동으로 시뮬레이션만 동작
    demo = CircleDrawerGUI(dual_mode=True)
    demo.run() 