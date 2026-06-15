# NX側で実行するスクリプト (Python 3.6 / 標準ライブラリのみ)
import socket
import struct
import sys

# --- UDP受信設定 ---
UDP_IP = "0.0.0.0"  # すべてのインターフェースから受け付ける
UDP_PORT = 8082

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# ==========================================================
# 【Unitree A1 制御用SDKのインポートと初期化】
# ここは機体にインストールされている unitree_legged_sdk の
# パスや環境（LCM等）に合わせて調整する必要があります。
# ==========================================================
# 例: sys.path.append('/home/unitree/unitree_legged_sdk/lib/python')
# import robot_interface (機体付属のラッパー)

print("NX側: コマンド受信サーバーを起動しました。ポート:", UDP_PORT)

try:
    while True:
        # PCからのデータ（12バイト）を受信
        data, addr = sock.recvfrom(12)

        # バイナリデータを3つのfloatに復元
        vx, vy, yaw = struct.unpack("fff", data)

        print(
            "受信コマンド -> 前後(vx): {:.2f}, 左右(vy): {:.2f}, 旋回(yaw): {:.2f}".format(
                vx, vy, yaw
            )
        )

        # ==========================================================
        # 【ここにA1のモーター・歩行命令への流し込みを行う】
        # 例:
        # high_cmd.velocity = [vx, vy]
        # high_cmd.yawSpeed = yaw
        # udp.SetSend(high_cmd)
        # udp.Send()
        # ==========================================================

except KeyboardInterrupt:
    print("\nサーバーを停止します。")
finally:
    sock.close()
