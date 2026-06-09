import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from pinky_bringup.pinky_battery import Battery


class Amr2BatteryPublisher(Node):
    def __init__(self):
        super().__init__("amr2_battery_publisher")

        self.declare_parameter("battery_full_voltage", 8.8)
        self.declare_parameter("battery_empty_voltage", 8.0)
        self.declare_parameter("battery_publish_period_sec", 5.0)

        self._battery = Battery()
        self._percent_pub = self.create_publisher(Float32, "battery/percent", 10)
        self._voltage_pub = self.create_publisher(Float32, "battery/voltage", 10)

        period = float(self.get_parameter("battery_publish_period_sec").value)
        self._timer = self.create_timer(period, self._publish_battery)

    def _publish_battery(self) -> None:
        voltage = self._battery.get_voltage()
        if voltage is None:
            self.get_logger().warning("Battery voltage read failed; skipping publish")
            return

        voltage_msg = Float32()
        voltage_msg.data = float(voltage)
        self._voltage_pub.publish(voltage_msg)

        full_voltage = float(self.get_parameter("battery_full_voltage").value)
        empty_voltage = float(self.get_parameter("battery_empty_voltage").value)
        if full_voltage <= empty_voltage:
            self.get_logger().error(
                "battery_full_voltage must be greater than battery_empty_voltage"
            )
            return

        percent = (voltage - empty_voltage) / (full_voltage - empty_voltage) * 100.0
        percent = max(0.0, min(100.0, percent))

        percent_msg = Float32()
        percent_msg.data = round(percent, 2)
        self._percent_pub.publish(percent_msg)

    def destroy_node(self):
        self._battery.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    publisher = Amr2BatteryPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
