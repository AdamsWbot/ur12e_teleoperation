"""S570 串口 → TCP 桥接（推送模式，Ctrl+C 可退出）
用法: python s570_bridge.py [left|right] 默认 left
"""
import socket
import serial
import struct
import sys
import time

PORT = 15570
ARM = 1 if len(sys.argv) < 2 or sys.argv[1] == "left" else 2

try:
    ser = serial.Serial("COM7", 1000000, timeout=0.05)
    print(f"S570 串口已打开 (COM7, arm={'left' if ARM==1 else 'right'})")
except Exception as e:
    print(f"串口打开失败: {e}")
    sys.exit(1)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("127.0.0.1", PORT))
server.listen(1)
server.settimeout(1)
print(f"桥接 127.0.0.1:{PORT}，等待连接...")

try:
    while True:
        try:
            conn, addr = server.accept()
        except socket.timeout:
            continue
        print(f"连接: {addr}")
        try:
            while True:
                cmd = bytes([0xFE, 0xFE, 0x03, 0x02, ARM, 0xFA])
                ser.reset_input_buffer()
                ser.write(cmd)

                head = ser.read(2)
                if head != b"\xfe\xfe":
                    continue
                data_len = ser.read(1)
                if not data_len:
                    continue
                remaining = ser.read(data_len[0])
                if not remaining:
                    continue

                payload = remaining[:-1]
                try:
                    conn.sendall(struct.pack("!H", len(payload)) + payload)
                except (ConnectionError, OSError):
                    break
        except (ConnectionError, OSError):
            pass
        finally:
            try:
                conn.close()
            except Exception:
                pass
        print("连接断开，等待重连...")
except KeyboardInterrupt:
    print("\n退出")
finally:
    server.close()
    ser.close()
