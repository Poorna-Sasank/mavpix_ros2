#!/usr/bin/env python3

import asyncio
import threading
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.drone = System()
        self.command_queue = queue.Queue()  # Thread-safe queue for commands
        self.loop = asyncio.new_event_loop()  # New asyncio event loop for MAVSDK
        self.mavsdk_thread = threading.Thread(target=self.run_mavsdk_loop, daemon=True)

        # ROS2 Publishers
        self.telemetry_pub = self.create_publisher(
            Float32MultiArray, 'drone/telemetry/velocity_ned', 10)
        self.position_pub = self.create_publisher(
            PoseStamped, 'drone/position_ned', 10)

        # Start MAVSDK async loop in a separate thread
        self.mavsdk_thread.start()
        self.get_logger().info("DroneNode initialized")

    def run_mavsdk_loop(self):
        """Run the MAVSDK asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run_mavsdk())

    async def run_mavsdk(self):
        """Main MAVSDK async logic with hardcoded setpoints."""
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
                break

        self.get_logger().info("Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Global position estimate OK")
                break

        # Start telemetry publishing
        asyncio.ensure_future(self.publish_telemetry())

        # Hardcoded sequence
        self.get_logger().info("-- Arming")
        await self.drone.action.arm()

        self.get_logger().info("-- Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.get_logger().error(f"Starting offboard failed: {error._result.result}")
            await self.drone.action.disarm()
            return

        self.get_logger().info("-- Go 0m North, 0m East, -10m Down")
        await self.drone.offboard.set_position_velocity_ned(
            PositionNedYaw(0.0, 0.0, -10.0, 0.0),
            VelocityNedYaw(0.0, 0.0, -1.0, 0.0))
        await asyncio.sleep(10)

        self.get_logger().info("-- Go 50m North, 0m East, -10m Down")
        await self.drone.offboard.set_position_velocity_ned(
            PositionNedYaw(50.0, 0.0, -10.0, 0.0),
            VelocityNedYaw(1.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(20)

        self.get_logger().info("-- Landing")
        try:
            await self.drone.offboard.stop()
        except OffboardError as error:
            self.get_logger().error(f"Stopping offboard failed: {error._result.result}")
        await self.drone.action.land()

    async def publish_telemetry(self):
        """Publish telemetry data to ROS2 topics."""
        async for odom in self.drone.telemetry.position_velocity_ned():
            # Publish velocity
            vel_msg = Float32MultiArray()
            vel_msg.data = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]
            self.telemetry_pub.publish(vel_msg)

            # Publish position
            pos_msg = PoseStamped()
            pos_msg.header.stamp = self.get_clock().now().to_msg()
            pos_msg.header.frame_id = "ned"
            pos_msg.pose.position.x = float(odom.position.north_m)
            pos_msg.pose.position.y = float(odom.position.east_m)
            pos_msg.pose.position.z = float(odom.position.down_m)
            self.position_pub.publish(pos_msg)

def main():
    rclpy.init()
    node = DroneNode()
    try:
        rclpy.spin(node)  # Run ROS2 in the main thread
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()