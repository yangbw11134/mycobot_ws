#!/usr/bin/env python3
"""
아두이노로 회전판만 360도 회전하는지 테스트하는 스크립트
- 카메라/녹화 없음
- 아두이노 직렬 연결 후 지정 각도만큼 회전 명령 전송
"""

import time
import glob
import argparse
import sys

try:
    import serial
except ImportError:
    print("❌ pyserial 모듈이 필요합니다. 설치: pip install pyserial")
    sys.exit(1)


def find_arduino_port(explicit_port: str | None = None) -> str | None:
    """아두이노 직렬 포트를 찾는다 (명시 포트가 있으면 그대로 사용)."""
    if explicit_port:
        return explicit_port

    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    if not ports:
        return None
    return ports[0]


def connect_arduino(port: str, baudrate: int = 9600, timeout: float = 1.0) -> serial.Serial | None:
    """아두이노와 직렬 연결을 맺는다."""
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # 보드 리셋 대기
        print(f"✅ 아두이노 연결됨: {port} @ {baudrate}bps")
        return ser
    except Exception as e:
        print(f"❌ 아두이노 연결 실패 ({port}): {e}")
        return None


def build_speed_command(speed: int, format_type: str = "S") -> bytes:
    """
    속도 설정 명령을 구성한다.
    speed: 스텝 속도 (예: 1000 = 1000 스텝/초 또는 1000 마이크로초 딜레이)
    format_type: 명령 형식 ("S" = "S1000", "s" = "s1000", "V" = "V1000" 등)
    """
    return f"{format_type}{speed:04d}\n".encode()


def build_step_command(steps: int) -> bytes:
    """6자리 프로토콜로 스텝 명령을 구성한다: [부호 1자리][스텝수 5자리]."""
    if steps >= 0:
        cmd = f"0{steps:05d}"
    else:
        cmd = f"1{abs(steps):05d}"
    return (cmd + "\n").encode()


def degrees_to_steps(degrees: float, gear_ratio: float, steps_per_rev: float) -> int:
    """
    각도를 스텝수로 변환.
    gear_ratio: 기어비 (모터 1회전 대비 출력축 1회전에 필요한 모터 회전수)
    steps_per_rev: 모터 1회전당 스텝 수
    """
    total_motor_revs = degrees / 360.0 * gear_ratio
    total_steps = int(round(total_motor_revs * steps_per_rev))
    return total_steps


def wait_for_done(ser: serial.Serial, timeout_sec: float) -> bool:
    """아두이노로부터 완료 신호(DONE/COMPLETE/FINISH/END)를 대기한다."""
    start = time.time()
    print("🔄 회전 완료 신호를 기다리는 중...")
    while True:
        if time.time() - start > timeout_sec:
            print(f"⏱️ 타임아웃 ({timeout_sec}초) - 완료 신호를 받지 못했습니다.")
            return False
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"📨 아두이노 메시지: {line}")
                    if any(k in line.upper() for k in ["DONE", "COMPLETE", "FINISH", "END"]):
                        print("✅ 회전 완료 신호 수신")
                        return True
        except Exception:
            pass
        time.sleep(0.05)


def run_rotation(
    degrees: float,
    gear_ratio: float,
    steps_per_rev: float,
    timeout_sec: float,
    port: str | None,
    speed: int | None = None,
) -> bool:
    found_port = find_arduino_port(port)
    if not found_port:
        print("❌ 아두이노 포트를 찾을 수 없습니다. USB 케이블과 권한을 확인하세요.")
        return False

    ser = connect_arduino(found_port)
    if not ser:
        return False

    try:
        steps = degrees_to_steps(degrees, gear_ratio, steps_per_rev)
        print(f"➡️  회전 명령 준비: {degrees}도 -> {steps} 스텝")

        # 버퍼 완전히 비우기
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)  # 버퍼 비우기 대기
        
        # 속도 설정 (지정된 경우)
        # 주의: 아두이노가 속도 명령을 지원하지 않으면 무시됩니다
        if speed is not None:
            print(f"⚙️  속도 설정 시도: {speed}")
            print(f"   ⚠️  속도 명령이 작동하지 않으면 아두이노 펌웨어를 확인하세요")
            
            # 속도 명령 전송 (단순 형식)
            speed_cmd = build_speed_command(speed, "S")
            print(f"   전송: {speed_cmd.decode().strip()}")
            ser.write(speed_cmd)
            ser.flush()
            
            # 짧은 대기 후 버퍼 확인
            time.sleep(0.1)
            
            # 아두이노 응답 확인 (있다면)
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"   아두이노 응답: {response}")
            
            # 버퍼 정리 (속도 명령이 문제를 일으킬 수 있으므로)
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.1)
        
        # 회전 명령 전송
        step_cmd = build_step_command(steps)
        print(f"✅ 회전 명령 전송: {step_cmd.decode().strip()}")
        ser.write(step_cmd)
        ser.flush()  # 버퍼 플러시
        print(f"✅ 회전 명령 전송 완료 ({steps} 스텝)")

        ok = wait_for_done(ser, timeout_sec)
        if ok:
            print("🎉  회전판 테스트 성공")
        else:
            print("⚠️  완료 신호 미수신 (회전은 되었을 수 있음)")
        return ok
    finally:
        try:
            ser.close()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="아두이노 회전판 360도 테스트")
    parser.add_argument("--degrees", type=float, default=360.0, help="회전 각도 (기본 360)")
    parser.add_argument("--gear_ratio", type=float, default=6.0, help="기어비 (기본 6.0)")
    parser.add_argument("--steps_per_rev", type=float, default=2048.0, help="모터 1회전 스텝 수 (기본 2048)")
    parser.add_argument("--timeout", type=float, default=120.0, help="완료 대기 타임아웃 초 (기본 120)")
    parser.add_argument("--port", type=str, default=None, help="직렬 포트 (예: /dev/ttyUSB0). 미지정 시 자동 탐색")
    parser.add_argument("--speed", type=int, default=None, help="회전 속도 설정 (낮을수록 빠름, 예: 500=빠름, 2000=느림). 미지정 시 아두이노 기본값 사용")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    print("\n🧪 회전판 단독 테스트를 시작합니다.")
    print(f"- 각도: {args.degrees}도, 기어비: {args.gear_ratio}, 스텝/회전: {args.steps_per_rev}")
    print(f"- 타임아웃: {args.timeout}초, 포트: {args.port or '자동 탐색'}")
    if args.speed:
        print(f"- 속도: {args.speed} (낮을수록 빠름)")
    else:
        print("- 속도: 아두이노 기본값 사용")
    print()

    success = run_rotation(
        degrees=args.degrees,
        gear_ratio=args.gear_ratio,
        steps_per_rev=args.steps_per_rev,
        timeout_sec=args.timeout,
        port=args.port,
        speed=args.speed,
    )

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()


