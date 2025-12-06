#!/usr/bin/env python3
"""
CAN 하드웨어 연결 테스트 런치파일

이 런치파일은 실제 모터 없이 CAN 인터페이스 연결만 테스트합니다.
MyActuator RMD 모터와의 통신 가능 여부를 확인하는 용도입니다.

사용법:
    ros2 launch myactuator_hardware test_connection.launch.py
    ros2 launch myactuator_hardware test_connection.launch.py can_interface:=vcan0
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def check_can_interface(context, *args, **kwargs):
    """CAN 인터페이스 상태를 확인하고 결과를 출력합니다."""
    can_interface = LaunchConfiguration('can_interface').perform(context)
    
    nodes = []
    
    # 헤더 출력
    nodes.append(LogInfo(msg="\n" + "="*70))
    nodes.append(LogInfo(msg="  MyActuator Hardware - CAN Connection Test"))
    nodes.append(LogInfo(msg="="*70))
    nodes.append(LogInfo(msg=f"  Testing CAN interface: {can_interface}"))
    nodes.append(LogInfo(msg="="*70 + "\n"))
    
    # CAN 인터페이스 확인
    try:
        # 인터페이스 존재 확인
        result = subprocess.run(
            ['ip', 'link', 'show', can_interface],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            # 인터페이스가 UP 상태인지 확인
            if 'UP' in result.stdout:
                nodes.append(LogInfo(msg=f"  ✓ CAN interface '{can_interface}' is UP and ready"))
                nodes.append(LogInfo(msg=""))
                
                # 상세 정보 출력
                for line in result.stdout.strip().split('\n'):
                    nodes.append(LogInfo(msg=f"    {line.strip()}"))
                
                nodes.append(LogInfo(msg=""))
                nodes.append(LogInfo(msg="  ✓ Connection test PASSED"))
                nodes.append(LogInfo(msg=""))
                nodes.append(LogInfo(msg="  Next steps:"))
                nodes.append(LogInfo(msg="    1. Connect your MyActuator RMD motors"))
                nodes.append(LogInfo(msg="    2. Power on the motors"))
                nodes.append(LogInfo(msg="    3. Run: ros2 launch fr_arm_moveit_config hardware.launch.py"))
            else:
                nodes.append(LogInfo(msg=f"  ✗ CAN interface '{can_interface}' exists but is DOWN"))
                nodes.append(LogInfo(msg=""))
                nodes.append(LogInfo(msg="  To bring it up:"))
                nodes.append(LogInfo(msg=f"    sudo ip link set {can_interface} up"))
        else:
            nodes.append(LogInfo(msg=f"  ✗ CAN interface '{can_interface}' not found!"))
            nodes.append(LogInfo(msg=""))
            
            # 사용 가능한 CAN 인터페이스 목록
            result_all = subprocess.run(
                ['ip', 'link', 'show'],
                capture_output=True,
                text=True
            )
            
            can_interfaces = []
            for line in result_all.stdout.split('\n'):
                if 'can' in line.lower() or 'vcan' in line.lower():
                    # 인터페이스 이름 추출
                    parts = line.split(':')
                    if len(parts) >= 2:
                        iface = parts[1].strip().split('@')[0]
                        if iface and iface not in can_interfaces:
                            can_interfaces.append(iface)
            
            if can_interfaces:
                nodes.append(LogInfo(msg="  Available CAN interfaces:"))
                for iface in can_interfaces:
                    nodes.append(LogInfo(msg=f"    • {iface}"))
            else:
                nodes.append(LogInfo(msg="  No CAN interfaces found!"))
            
            nodes.append(LogInfo(msg=""))
            nodes.append(LogInfo(msg="  Setup instructions:"))
            nodes.append(LogInfo(msg=""))
            nodes.append(LogInfo(msg="  For real CAN hardware:"))
            nodes.append(LogInfo(msg=f"    sudo ip link set {can_interface} type can bitrate 1000000"))
            nodes.append(LogInfo(msg=f"    sudo ip link set {can_interface} up"))
            nodes.append(LogInfo(msg=""))
            nodes.append(LogInfo(msg="  For virtual CAN (testing):"))
            nodes.append(LogInfo(msg="    sudo modprobe vcan"))
            nodes.append(LogInfo(msg="    sudo ip link add dev vcan0 type vcan"))
            nodes.append(LogInfo(msg="    sudo ip link set vcan0 up"))
            
    except Exception as e:
        nodes.append(LogInfo(msg=f"  ✗ Error checking CAN interface: {str(e)}"))
    
    nodes.append(LogInfo(msg=""))
    nodes.append(LogInfo(msg="="*70 + "\n"))
    
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # 런치 인수 선언
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name (e.g., can0, vcan0)'
        ),
        
        # CAN 인터페이스 체크 실행
        OpaqueFunction(function=check_can_interface),
    ])
