import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from dynamixel_controller.msg import DynamixelController  # C++ノードで定義したメッセージをインポート
from array import array

class DynamixelControllerClient(Node):
    ids = [0, 2] #, 3, 4, 5, 6, 7]
    def __init__(self):
        super().__init__('dynamixel_controller_client')
        
        # 命令送信用トピック
        self.tx_publisher = self.create_publisher(UInt8MultiArray, 'dynamixel_tx', 10)
        
        msg = UInt8MultiArray()
        msg.data = array('B', [DynamixelController.SYNC_WRITE, DynamixelController.OPERATING_MODE, 1]) # [SYNC_WRITE, アドレス, データ長, ID, モード]
        for dxl_id in self.ids:
            msg.data += array('B', [dxl_id, 3])  # Position Controlモード3
        self.tx_publisher.publish(msg)
        
        msg = UInt8MultiArray() # msgの再初期化
        msg.data = array('B', [DynamixelController.SYNC_WRITE, DynamixelController.TORQUE_ENABLE, 1]) # [SYNC_WRITE, アドレス, データ長, ID, トルクON]
        for dxl_id in self.ids:
            msg.data += array('B',[dxl_id, 1])  # トルクを有効にする
        self.tx_publisher.publish(msg)
        
        # C++ノードからの応答受信用トピック
        self.rx_subscription = self.create_subscription(
            UInt8MultiArray,
            'dynamixel_rx',
            self.rx_callback,
            10
        )
        # 5秒ごとに SYNC_WRITE 命令, 1秒ごとに SYNC_READ 命令を送信するタイマー
        self.write_timer = self.create_timer(5.0, self.write_callback)
        self.read_timer = self.create_timer(1.0, self.read_callback)
        
    def write_callback(self):
        # SYNC_WRITE 命令のフォーマット: [SYNC_WRITE, goal_address, data_length, id(1), joint_position(1)]
        msg = UInt8MultiArray()
        SYNC_WRITE = 131   # DynamixelController.msg で定義した SYNC_WRITE (0x83)
        goal_address = 116  # 目標位置アドレス
        data_length = 4
        
        def encode_goal_position(dxl_id, position):
        # 位置を4バイトのリトルエンディアンに変換
            pos_bytes = position.to_bytes(4, 'little')
            return [dxl_id] + list(pos_bytes)

        joint_positions = {
            0: 512, # joint1 (TTL, XM540)
            2: 400 # joint2 (TTL, XM540)
            # 3: 300, # joint3 (RS485, XL430)
            # 4: 600, # joint4 (RS485, XL430)
            # 5: 200, # joint5 (RS485, XL430)
            # 6: 128, # joint6 (RS485, XL430)
            # 7: 900  # joint7 (RS485, XL430)
        }

        msg.data = array('B', [SYNC_WRITE, goal_address, data_length])
        for dxl_id in self.ids:
            msg.data += array('B', encode_goal_position(dxl_id, joint_positions[dxl_id]))
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Publisher SYNC_WRITE command: {msg.data}')
        
    def read_callback(self):
        # SYNC_READ 命令のフォーマット: [SYNC_READ, start_address, data_length, id1, id2, ...]
        msg = UInt8MultiArray()
        SYNC_READ = 130     # DynamixelController.msg で定義した SYNC_READ (0x82)
        start_address = 132   # PRESENT_POSITION アドレス
        data_length = 4
        
        msg.data = array('B', [SYNC_READ, start_address, data_length] + self.ids)
        self.tx_publisher.publish(msg)
        self.get_logger().info(f'Published SYNC_READ command: {msg.data}')
        
    def rx_callback(self, msg):
        # msg = UInt8MultiArray()
        self.get_logger().info(f'Received response: {msg.data}')
        # SYNC_READ = DynamixelController.SYNC_READ
        # if not msg.data:
        #     self.get_logger().warn("Empty response received.")
        #     return
        
        # if msg.data[0] != SYNC_READ:
        #     self.get_logger().warn(f"Received unknown response: {msg.data}")
        #     return

        # # 2つ目以降がレスポンスデータ（エスケープ処理済み）→順に解釈
        # data = msg.data[1:]
        # num_motors = len(self.ids)
        # expected_length = num_motors * 4  # 1モーターあたり4バイト

        # if len(data) < expected_length:
        #     self.get_logger().warn(f"Not enough data: got {len(data)} bytes, expected {expected_length}")
        #     return

        # for i, dxl_id in enumerate(self.ids):
        #     offset = i * 4
        #     position_bytes = data[offset:offset+4]
        #     position_value = int.from_bytes(position_bytes, byteorder='little')
        #     angle_deg = (position_value % 4096) * 360.0 / 4096
        #     self.get_logger().info(f"ID {dxl_id}: position = {position_value}, angle ≈ {angle_deg:.2f}°")

        
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControllerClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()