#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class PoseFollower(Node):
    def __init__(self):
        super().__init__('turtle_pose_follower')

        # Подписчик на /turtle1/pose
        self.pose_subscriber = self.create_subscription(
            Float64MultiArray,
            '/odom',
            self.pose_callback,
            10
        )
        

        self.pose_subscriber = self.create_subscription(
            Float64MultiArray,
            '/target',
            self.set_goal,
            10
        )

        # Издатель в топик /turtle1/cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Hello, world!")
        # Текущая поза черепашки
        self.current_pose = Float64MultiArray()

        # Таймер управления (10 Гц = 0.1 сек)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Цель (перезаполняется каждый раз через set_goal)
        self.x_goal = 0.0
        self.y_goal = 0.0

        # Коэффициенты «П»-регулятора
        self.linear_k = 1.0
        self.angular_k = 4.0

        # Порог «прибыли» к цели
        self.arrival_tolerance = 10.0

        # Флаги состояния
        self.reached = False
        self.unreachable = False
        
        self.task = False

        # Переменные для отслеживания «застрявшей» черепашки
        self.last_distance = float('inf')
        self.last_progress_time = self.get_clock().now().nanoseconds
        self.no_progress_limit = 10.0  # 2 секунды без движения— объявляем недостижимость

    def set_goal(self, msg: Float64MultiArray):
        """Задать новую цель и сбросить состояние."""
        #self.current_pose = msg
        
        self.x_goal = msg.data[0]
        self.y_goal = msg.data[1]
        
        #self.get_logger().info(f"{self.x_goal}")

        self.reached = False
        self.unreachable = False

        self.last_distance = float('inf')
        self.last_progress_time = self.get_clock().now().nanoseconds
        
        self.task = True
    def pose_callback(self, msag: Float64MultiArray):
        """Сохранить текущую позу черепашки."""
        self.current_pose = msag
        #self.get_logger().info(f"{msag}")
    def control_loop(self):
        """
        Вызывается таймером каждые 0.1 сек.
        Управляет движением к цели или определяет, что цель недостижима.
        """
        # Если уже «финиш» или «застряли», ничего не делаем:
        if self.reached or self.unreachable or not self.task:
            return

        # Вычисляем расстояние до цели
        dx = self.x_goal - self.current_pose.data[0]
        dy = self.y_goal - self.current_pose.data[1]
        distance = math.sqrt(dx*dx + dy*dy)
        #self.get_logger().info(f"dx: {dx}     dy: {dy}")
        # Проверяем, достигли ли мы цели
        if distance < self.arrival_tolerance:
            #self.get_logger().info(f"{distance}")
            self.stop_moving()
            self.reached = True
            self.get_logger().info('Goal reached! Stopping.')
            return

        # Проверяем, есть ли прогресс(движение). Смотрим на уменьшение distance
        current_time = self.get_clock().now().nanoseconds
        if distance < self.last_distance - 0.001:
            # Считаем, что движение есть, обновим метки
            self.last_progress_time = current_time
            self.last_distance = distance
        else:
            # Движения нет. Сколько прошло времени?
            time_no_progress = (current_time - self.last_progress_time) / 1e9
            if time_no_progress > self.no_progress_limit:
                # Долго стоим (или крутимся на месте) без движения — объявляем недостижимость
                self.stop_moving()
                self.unreachable = True
                self.get_logger().warn('Goal is unreachable! Stopping.')
                return

        # Управление по «П»-регулятору
        desired_angle = math.atan2(dy, dx)
        # Считаем угол вектора в заданной точке
        angle_error = desired_angle - self.current_pose.data[2]
        self.get_logger().info(f"desired_angle: {desired_angle}   angle_error: {angle_error}")
        # Находим ошибку - разницу между углом черепахи и вектором

        # Нормализуем угол в диапазон [-pi, pi]. Чтобы поворачиваться в ближайшую сторону
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Формируем Twist
        twist = Twist()

        # Линейная скорость (ограничиваем сверху)
        twist.linear.x = self.linear_k * distance
        self.get_logger().info(f"{twist.linear.x}")
        if twist.linear.x > 0.1:
            twist.linear.x = 0.1

        # Угловая скорость (аналогично ограничиваем)
        twist.angular.z = self.angular_k * angle_error
        if twist.angular.z > 0.1:
            twist.angular.z = 0.1
        elif twist.angular.z < -0.1:
            twist.angular.z = -0.1
        
        twist.linear.x = twist.linear.x * -1
        # Публикуем
        self.publisher_.publish(twist)
        
        self.task = False
    def stop_moving(self):
        """Опубликовать нулевые скорости (остановка)."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
