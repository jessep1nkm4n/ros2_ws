import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool # Detector node'dan gelen durak algılama için
from robotaksi_interfaces.msg import RobotState # RobotStateController'a durum değişikliği göndermek için

import time
import math

class StopSignAlgorithm(Node):

    def __init__(self):
        super().__init__('stop_sign_algorithm')

        # Publisher'ları yeni kontrolörlere göre ayarla
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # RobotStateController'a durum değişikliği göndermek için publisher
        self.robot_state_publisher = self.create_publisher(RobotState, '/robotaksi/robot_state_command', 10)

        # --- DURAK ALGILAMA SUBSCRIBER'I ---
        # detector_node'un durak algıladığında yayınladığı topic'i dinle.
        self.bus_stop_subscription_ = self.create_subscription(
            Bool,
            '/bus_stop_detected_signal',  # TrafficSignDetector'daki yeni topic adı
            self.bus_stop_callback, # Mesaj geldiğinde çağrılacak fonksiyon
            10
        )
        self.get_logger().info('Stop Sign Algorithm Node Started with separate controllers and subscribed to /bus_stop_detected_signal.')

        # Robotun hareket parametreleri
        self.base_linear_speed = 1.0  # m/s (İleri hız)
        self.base_angular_position = math.radians(15) # rad (Direksiyonun maksimum açısı, yumuşak dönüş için)
        self.wheel_radius = 0.1 # Metre cinsinden tekerlek yarıçapı. **KENDİ ROBOT MODELİNİZE GÖRE AYARLAYIN!**
                                # Yanlışsa, tekerlek hızı yanlış hesaplanır.

        # Rampalama için sabitler
        self.ramp_steps = 20  # Rampalama için adım sayısı (daha fazla adım = daha yumuşak geçiş)
        self.ramp_interval = 0.05 # Saniye cinsinden her adım arasındaki bekleme süresi

        # Durak sekansının zaten çalışıp çalışmadığını kontrol etmek için bayraklar
        self.stop_sequence_active = False # Dizinin şu an çalışıp çalışmadığını gösterir
        self.has_executed_bus_stop_sequence = False # Durak sekansını bir kez çalıştırdığımızı işaretlemek için

    def bus_stop_callback(self, msg: Bool):
        """
        Detector node'dan gelen durak algılama mesajlarını işler.
        'True' mesajı alındığında ve sekans aktif değilse, durak sekansını başlatır.
        """
       
        if msg.data and not self.stop_sequence_active and not self.has_executed_bus_stop_sequence:
            self.get_logger().info('Durak tabelası algılandı! Sekans başlatılıyor...')
            self.stop_sequence_active = True        # Sekansın aktif olduğunu işaretle
            self.has_executed_bus_stop_sequence = True # Sekansı bir kez tetiklediğimizi işaretle

            # RobotStateController'a robotun durması gerektiğini bildir.
            # TrafficSignDetector kodunda 'STOPPING_AT_PEDESTRIAN_CROSSING' kullanılıyordu.
            # Genel bir 'STOPPED' veya 'STOPPING_AT_BUS_STOP' durumu tanımlayabiliriz.
            # Mevcut RobotStateController'daki "STOPPING_AT_PEDESTRIAN_CROSSING" durumunu kullanabiliriz.
            # Veya yeni bir durum adı tanımlayabiliriz. Biz şimdilik daha genel bir isim kullanalım.
            self.publish_robot_state("STOPPING") # RobotStateController'a durma komutu gönder
            self.get_logger().info("RobotStateController'a 'STOPPING' durumu gönderildi.")


            # Durak tabelası sekansını başlat
            self.execute_stop_sign_sequence()

            # Sekans tamamlandığında bayrakları sıfırla ve robotun şerit takibine dönmesini sağla
            self.stop_sequence_active = False
            self.has_executed_bus_stop_sequence = False # Sekans bitti, tekrar tetiklenebilir

            # RobotStateController'a robotun şerit takibine geri dönebileceğini bildir
            self.publish_robot_state("LANE_FOLLOWING") # RobotStateController'a şerit takibine dön komutu gönder
            self.get_logger().info("RobotStateController'a 'LANE_FOLLOWING' durumu gönderildi. Durak sekansı tamamlandı.")

        elif msg.data and self.stop_sequence_active:
            self.get_logger().info('Durak tabelası algılandı, ancak sekans zaten aktif. Şimdilik yok sayılıyor.')
        elif msg.data and self.has_executed_bus_stop_sequence:
            self.get_logger().info('Durak tabelası algılandı, ancak sekans zaten bir kez çalıştırıldı. Yok sayılıyor.')

    def publish_robot_state(self, state: str):
        """RobotStateController'a robotun durumunu yayınlar."""
        msg = RobotState()
        msg.state = state
        self.robot_state_publisher.publish(msg)
        self.get_logger().info(f"Robot Durumu Yayınlandı: {state}")


    def publish_steering_position(self, left_angle, right_angle):
        """Direksiyon eklemlerinin pozisyonunu yayınlar."""
        msg = Float64MultiArray()
        msg.data = [float(left_angle), float(right_angle)]
        self.position_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Steering Positions: Left={math.degrees(left_angle):.2f} deg, Right={math.degrees(right_angle):.2f} deg')

    def publish_wheel_velocity(self, left_velocity, right_velocity):
        """Arka tekerleklerin açısal hızını (rad/s) yayınlar."""
        msg = Float64MultiArray()
        msg.data = [float(left_velocity), float(right_velocity)]
        self.velocity_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Wheel Velocities: Left={left_velocity:.2f} rad/s, Right={right_velocity:.2f} rad/s')

    def move_forward(self, distance_meters):
        """Belirtilen mesafe kadar ileri gider."""
        self.get_logger().info(f'Moving forward {distance_meters} meters...')

        # Direksiyonu düz tut
        self.publish_steering_position(0.0, 0.0)
        time.sleep(0.1) # Direksiyonun düzelmesi için kısa bekleme

        # Lineer hızı açısal tekerlek hızına çevir: v = w * r => w = v / r
        target_wheel_velocity = self.base_linear_speed / self.wheel_radius
        duration = distance_meters / self.base_linear_speed

        self._ramp_wheel_velocity(target_wheel_velocity, duration)
        self.stop_robot()

    def turn(self, target_angle_degrees, turn_distance_meters=0.0, ramp_duration_s=1.0):
        """
        Belirtilen açıya direksiyonu döndürür ve bu sırada ileri hareket eder.
        Sağ için negatif açı, sol için pozitif açı.
        turn_distance_meters: Dönüş sırasında katedilecek yaklaşık lineer mesafe.
        ramp_duration_s: Direksiyonun hedeflenen açıya ulaşması için geçen süre.
        """
        self.get_logger().info(f'Turning to {target_angle_degrees} degrees while moving {turn_distance_meters} meters with ramp_duration={ramp_duration_s}s...')

        target_angle_radians = math.radians(target_angle_degrees)

        # Direksiyonu hedeflenen açıya yumuşakça getir
        self._ramp_steering_angle(target_angle_radians, target_angle_radians, ramp_duration_s)

        # Tekerlekleri döndürerek ileri hareket et
        target_wheel_velocity = self.base_linear_speed / self.wheel_radius
        duration_for_turn = 0.0
        if self.base_linear_speed != 0:
            duration_for_turn = turn_distance_meters / self.base_linear_speed

        # Direksiyon hedeflenen açıda tutulurken tekerlekleri döndür.
        self._ramp_wheel_velocity(target_wheel_velocity, duration_for_turn, initial_ramp_done=True)

        self.stop_robot() # Dönüş bitince durdur

    def _ramp_steering_angle(self, target_left_angle, target_right_angle, ramp_duration_s):
        """Direksiyon eklemlerini belirli bir süre içinde kademeli olarak hedeflenen açıya getirir."""
        start_left_angle = 0.0 # Varsayılan olarak düz pozisyondan başla
        start_right_angle = 0.0

        num_steps = int(ramp_duration_s / self.ramp_interval)
        if num_steps == 0: num_steps = 1 # En az bir adım

        for i in range(num_steps + 1):
            factor = i / num_steps
            current_left_angle = start_left_angle + (target_left_angle - start_left_angle) * factor
            current_right_angle = start_right_angle + (target_right_angle - start_right_angle) * factor
            self.publish_steering_position(current_left_angle, current_right_angle)
            time.sleep(self.ramp_interval)
        # Son pozisyonda kalması için bir kez daha yayınla
        self.publish_steering_position(target_left_angle, target_right_angle)

    def _ramp_wheel_velocity(self, target_velocity, total_duration_s, initial_ramp_done=False):
        """Tekerleklerin hızını belirli bir süre içinde kademeli olarak artırır ve sonra yavaşça durdurur."""
        start_velocity = 0.0 if not initial_ramp_done else target_velocity

        num_steps = int(total_duration_s / self.ramp_interval)
        if num_steps == 0: num_steps = 1

        ramp_up_steps = num_steps // 2
        cruise_steps = num_steps - ramp_up_steps * 2

        # Hızı artırma rampası
        for i in range(ramp_up_steps + 1):
            factor = i / ramp_up_steps if ramp_up_steps > 0 else 1.0
            current_velocity = start_velocity + (target_velocity - start_velocity) * factor
            self.publish_wheel_velocity(current_velocity, current_velocity)
            time.sleep(self.ramp_interval)

        # Hedef hızda seyir
        if cruise_steps > 0:
            self.publish_wheel_velocity(target_velocity, target_velocity)
            time.sleep(cruise_steps * self.ramp_interval)

        # Hızı azaltma rampası
        for i in range(ramp_up_steps + 1):
            factor = i / ramp_up_steps if ramp_up_steps > 0 else 1.0
            current_velocity = target_velocity - (target_velocity - 0.0) * factor
            self.publish_wheel_velocity(current_velocity, current_velocity)
            time.sleep(self.ramp_interval)
        self.publish_wheel_velocity(0.0, 0.0) # Son duruş


    def stop_robot(self):
        """Robotu durdurur."""
        self.get_logger().info('Stopping robot.')
        self.publish_steering_position(0.0, 0.0) # Direksiyonu sıfırla
        self.publish_wheel_velocity(0.0, 0.0) # Tekerlek hızlarını sıfırla
        time.sleep(0.5) # Durma komutlarının işlenmesi için yeterli bekleme

    def wait_for_seconds(self, seconds):
        """Belirtilen saniye kadar bekler."""
        self.get_logger().info(f'Waiting for {seconds} seconds...')
        self.stop_robot() # Beklerken robotun durduğundan emin ol
        time.sleep(seconds)
        self.get_logger().info('Wait finished.')

    def execute_stop_sign_sequence(self):
        """Durak tabelası görüldükten sonraki hareket dizisini çalıştırır."""
        self.get_logger().info('Executing stop sign sequence...')

        # PARK ETME DİZİSİ - BU KISIMDAKİ MESAFELERİ ROBOTUN YENİ DİNAMİKLERİNE GÖRE TEKRAR AYARLAMAN GEREKECEK.
        # Şu anki hızlar (1.0 m/s) ile önceki mesafeler çalışıyordu, ancak rampalama ek süreler getirecek.

        # 1. 1.0 metre ilerle
        self.move_forward(1.0)

        # 2. 5 derece sağa kır ve 6.5 metre ilerle (yumuşak dönüş ile)
        # Direksiyon açısını negatif veriyoruz (sağ), direksiyon açısı pozisyonu ve lineer hız
        self.turn(-15.0, turn_distance_meters=6.5, ramp_duration_s=0.7) # ramp_duration_s'yi ayarla

        # 3. 4.0 metre ilerle
        self.move_forward(3.5)

        # 4. 10 derece sola kır ve 7.0 metre ilerle (yumuşak dönüş ile)
        self.turn(30.0, turn_distance_meters=3.0, ramp_duration_s=0.7)

        # 5. 3.0 metre ilerle
        self.move_forward(3.0)

        # 6. 30 saniye bekle
        self.wait_for_seconds(30)

        # 7. 5 derece sola kır ve 6.5 metre ilerle (yumuşak dönüş ile)
        self.turn(15.0, turn_distance_meters=6.5, ramp_duration_s=0.7)

        # 8. 7.0 metre ilerle
        self.move_forward(5.0)

        # 9. 10 derece sağa kır ve 6.5 metre ilerle (yumuşak dönüş ile)
        self.turn(-30.0, turn_distance_meters=3.0, ramp_duration_s=0.7)

        # 10. Devam et
        self.get_logger().info('Stop sign sequence completed. Robot ready for next task.')
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = StopSignAlgorithm()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()