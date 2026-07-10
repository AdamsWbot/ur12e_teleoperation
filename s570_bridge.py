"""S570 串口 → TCP 桥接（在 Windows 上运行，供 WSL 读取）"""
import socket
import serial
import struct
import sys

PORT = 15570  # 给 WSL 连接的 TCP 端口

try:
    ser = serial.Serial("COM7", 1000000, timeout=0.1)
    print(f"S570 串口已打开 (COM7)")
except Exception as e:
    print(f"串口打开失败: {e}")
    sys.exit(1)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("127.0.0.1", PORT))
server.listen(1)
print(f"TCP 桥接已启动 127.0.0.1:{PORT}，等待 WSL 连接...")
conn, addr = server.accept()
print(f"WSL 已连接: {addr}")

try:
    while True:
        # 发送命令让 S570 回传数据
        cmd = bytes([0xFE, 0xFE, 0x03, 0x02, 0x01, 0xFA])
        ser.reset_input_buffer()
        ser.write(cmd)

        # 读响应帧头
        head = ser.read(2)
        if head != b"\xfe\xfe":
            continue
        data_len = ser.read(1)
        if not data_len:
            continue
        remaining = ser.read(data_len[0])
        if not remaining:
            continue

        # 打包发送给 WSL: 长度(2B) + 数据
        payload = remaining[:-1]  # 去掉尾 0xFA
        conn.sendall(struct.pack("!H", len(payload)) + payload)

except (KeyboardInterrupt, ConnectionError):
    print("\n退出")
finally:
    conn.close()
    server.close()
    ser.close()
