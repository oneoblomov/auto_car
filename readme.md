# ROS2

## 1. Nodes (Düğümler)

Her ana işlev için bir ROS 2 node'u vardır:

- **Sensör Düğümleri**
  - `get_lidar`: 2D Lidar verisi yayınlar.
  - `get_imu`: IMU verisi yayınlar.
  - `get_encoder`: Odometri verisi yayınlar.
  - `get_rgbd_camera`: RGB-D kamera renk ve derinlik görüntülerini yayınlar.
  - `get_targeting_camera`: Nişangah kamerası görüntüsü yayınlar.

- **Perception (Algılama) Düğümleri**
  - `detect_sign`: Tabela algılama ve sınıflandırma.
  - `detect_object`: YOLO ile nesne/hedef tespiti.
  - `segment_color`: Renk segmentasyonu ile işaret tespiti.
  - `detect_obstacle`: Lidar ve kamera ile engel tespiti.

- **Planning (Planlama) Düğümleri**
  - `plan_local_path`: DWA algoritması ile lokal yol planlama.
  - `manage_behavior`: Tabela ve hedeflere göre davranış yönetimi.
  - `manage_mode`: Sistem modları arası geçiş ve yönetim (Tam-otonom, Yarı-otonom, Manuel).

- **Control (Kontrol) Düğümleri**
  - `control_motor`: Motorlara hız komutu gönderir.
  - `control_servo`: Servo/lazer işaretleyici kontrolü.
  - `control_laser`: Lazer işaretleyici servo komutu.
  - `control_shooting`: Hedef tespiti ve atış kontrolü.

- **UI (Kullanıcı Arayüzü) Düğümleri**
  - `ui_web_dashboard`: Web tabanlı gösterge paneli.
  - `ui_remote_control`: Uzaktan manuel kontrol.
  - `ui_visualization_bridge`: RViz2 görselleştirmesi için köprü.
  - `ui_emergency_handler`: Acil durum yönetimi.

---

## 2. Topics (Konular)

Veri akışı için kullanılan ana ROS 2 topic'leri:

| Topic Adı                             | Mesaj Tipi                        | Açıklama                                 | Yayıncı Node           | Abone Node(lar)         |
|----------------------------------------|------------------------------------|------------------------------------------|------------------------|-------------------------|
| `/get_lidar/scan`                     | `sensor_msgs/LaserScan`            | Lidar verisi                             | get_lidar              | detect_obstacle, plan_local_path |
| `/get_imu/data`                       | `sensor_msgs/Imu`                  | IMU verisi                               | get_imu                | plan_local_path, manage_behavior |
| `/get_encoder/odom`                   | `nav_msgs/Odometry`                | Encoder odometrisi                       | get_encoder            | plan_local_path, manage_behavior |
| `/get_rgbd_camera/color_image`         | `sensor_msgs/Image`                | RGB-D kamera renk görüntüsü               | get_rgbd_camera        | detect_sign, detect_object, segment_color |
| `/get_rgbd_camera/depth_image`         | `sensor_msgs/Image`                | RGB-D kamera derinlik görüntüsü           | get_rgbd_camera        | detect_obstacle, control_shooting |
| `/get_targeting_camera/image`          | `sensor_msgs/Image`                | Nişangah kamera görüntüsü                | get_targeting_camera   | detect_object, control_shooting |
| `/detect_sign/signs`                  | `custom_msgs/SignArray`            | Algılanan tabelalar                      | detect_sign            | manage_behavior        |
| `/detect_object/targets`              | `custom_msgs/TargetInfo`           | Algılanan hedefler (YOLO)                | detect_object          | manage_behavior, control_shooting |
| `/segment_color/colors`               | `std_msgs/Float32MultiArray`       | Renk segmentasyonu sonuçları             | segment_color          | manage_behavior        |
| `/plan_local_path/cmd_vel`            | `geometry_msgs/Twist`              | Hız komutu (planlayıcı/operatör)         | plan_local_path, ui_remote_control, manage_behavior | control_motor, control_laser |
| `/control_servo/command`              | `std_msgs/Float64`                 | Servo/lazer işaretleyici komutu          | control_servo, control_shooting | donanım arayüzü        |
| `/control_laser/command`              | `std_msgs/Float64`                 | Lazer işaretleyici servo komutu          | control_laser, control_shooting | donanım arayüzü         |
| `/manage_behavior/robot_status`       | `std_msgs/String`                  | Robotun genel durumu                     | manage_behavior        | ui_web_dashboard           |
| `/manage_mode/system_mode`            | `std_msgs/Int8`                    | Sistem modu (0:Otonom, 1:Yarı, 2:Manuel) | manage_mode            | manage_behavior, ui_web_dashboard, control_motor |
| `/detect_obstacle/obstacle_map`       | `nav_msgs/OccupancyGrid`           | Engel haritası                           | detect_obstacle        | plan_local_path             |
| `/manage_behavior/terrain_status`     | `custom_msgs/TerrainInfo`          | Arazi durum bilgisi (IMU tabanlı)        | manage_behavior        | plan_local_path, control_motor |

---

## 3. Services (Servisler)

Servisler, anlık istek/yanıt gerektiren işlemler için kullanılır. Örnekler:

- **Mod Değişimi Servisi**
  - `/manage_mode/change_mode` (`custom_srvs/SetMode`): Robotun çalışma modunu değiştirir (otonom, yarı-otonom, manuel).
  - Sunucu: manage_mode
  - İstemci: ui_web_dashboard, ui_remote_control, manage_behavior

- **Acil Durdurma Servisi**
  - `/ui_emergency_handler/emergency_stop` (`std_srvs/Trigger`): Robotu acil olarak durdurur.
  - Sunucu: ui_emergency_handler
  - İstemci: ui_web_dashboard, ui_remote_control, detect_obstacle

- **Atış Kontrolü Servisi**  
  - `/control_shooting/start_shooting_sequence` (`custom_srvs/ShootingControl`): Atış dizisini başlatır.
  - Sunucu: control_shooting
  - İstemci: manage_behavior, ui_remote_control

- **Kalibrasyon Servisi**
  - `/get_sensors/calibrate` (`std_srvs/Trigger`): Sensör kalibrasyonu yapar.
  - Sunucu: get_sensors
  - İstemci: ui_web_dashboard

---

## 4. Actions (Eylemler)

Uzun süren ve takip gerektiren görevler için kullanılır:

- **Navigasyon Eylemi**
  - `/plan_local_path/navigate_to_point` (`custom_actions/NavigateToPoint`): Belirli bir hedefe gitme.
  - Sunucu: plan_local_path
  - İstemci: manage_behavior

- **Atış Eylemi**
  - `/control_shooting/shoot_target` (`custom_actions/ShootTarget`): Hedefe yönlenme, lazer işaretleme ve atış işlemi.
  - Sunucu: control_shooting
  - İstemci: manage_behavior, ui_remote_control

- **Davranış Yürütme Eylemi**
  - `/manage_behavior/execute_behavior` (`custom_actions/ExecuteBehavior`): Özel bir davranış modunu yürütür (ör: taşlı yolda ilerleme).
  - Sunucu: manage_behavior
  - İstemci: manage_mode

---

## 5. Mesaj Tipleri

### Standart Mesajlar

- `sensor_msgs/LaserScan` - Lidar tarama verisi
- `sensor_msgs/Imu` - IMU verileri (ivme, açısal hız, yönelim)
- `nav_msgs/Odometry` - Tekerlek odometrisi
- `sensor_msgs/Image` - Kamera görüntüleri
- `geometry_msgs/Twist` - Hız komutları
- `std_msgs/Float64` - Servo/lazer komutu
- `std_msgs/String` - Durum metinleri
- `std_msgs/Int8` - Mod bilgisi
- `nav_msgs/OccupancyGrid` - Engel haritası

### Custom Mesajlar

- `custom_msgs/SignArray` - Algılanan tabelaların listesi

``` c++
struct Sign {
  int id;
  string type;
  float confidence;
  geometry_msgs/Point position;
}
```

- `custom_msgs/TargetInfo` - Hedef bilgisi

``` c++
struct TargetInfo {
  int id;
  geometry_msgs/Point position;
  float distance;
  float angle;
  float confidence;
}
```

- `custom_msgs/TerrainInfo` - Arazi durum bilgisi

``` c++
struct TerrainInfo {
  int terrain_type;  
  float incline_x;   
  float incline_y;   
  float roughness;   
}
```

### Custom Servisler

- `custom_srvs/SetMode`

``` c++
int8 mode  # 0:Otonom, 1:Yarı-otonom, 2:Manuel
---
bool success
string message
```

- `custom_srvs/ShootingControl`

``` c++
bool enable
int8 target_id  # -1: otomatik hedef seçimi
---
bool success
int8 selected_target_id
```

---

## 6. Mod Geçiş Yönetimi

Sistem üç ana çalışma moduna sahiptir:

1. **Tam Otonom Mod** - Sistem tamamen otonom karar verir ve hareket eder.
2. **Yarı-Otonom Mod** - Sistem çoğunlukla otonom çalışır, atış bölgesi gibi özel durumlarda operatör kontrolüne geçer.
3. **Manuel Kontrol Modu** - Operatör doğrudan robotu kontrol eder, güvenlik sistemleri aktiftir.

Mod geçişleri aşağıdaki durumlarda tetiklenir:

- **Kullanıcı İsteği**: Web arayüzü veya uzaktan kontrol üzerinden
- **Özel Durum Tespiti**: Atış bölgesi tespitinde yarı-otonom moda geçiş
- **Hata Durumu**: Sensör arızası veya beklenmeyen durumda güvenli moda geçiş

`manage_mode` node'u, bu geçişleri yönetir ve ilgili node'lara bildirir. Geçiş sırasında güvenlik kontrolleri yapılır.

## 7. Özet Akış

- **Sensörler** → get_* düğümleri → Topic'ler üzerinden veri yayını
- **Algılama** → detect_*/segment_* düğümleri → Tabela, hedef, engel tespiti
- **Karar Verme** → manage_mode ve manage_behavior → Davranış seçimi
- **Planlama** → plan_local_path ve manage_behavior → Hız ve yön komutları
- **Kontrol** → control_* düğümleri → Aktüatörlere komut
- **Arayüz** → ui_* düğümleri → Kullanıcıya bilgi ve kontrol

---

## 8. Parkur Adımları İçin Kullanılan Node ve Algoritmalar

| Parkur Adımı            | Kullanılan Nodes                        | Topic'ler                                         | Algoritmalar                      |
|-------------------------|-----------------------------------------|--------------------------------------------------|----------------------------------|
| Dik Engel               | detect_obstacle, plan_local_path        | /get_lidar/scan, /get_rgbd_camera/depth_image, /plan_local_path/cmd_vel | Engel tespiti, DWA planlama     |
| Taşlı Çakıllı Yol       | detect_sign, manage_behavior            | /detect_sign/signs, /get_imu/data, /manage_behavior/terrain_status | Tabela tanıma, hız optimizasyonu |
| Yan Eğim                | get_imu, manage_behavior                | /get_imu/data, /detect_sign/signs, /plan_local_path/cmd_vel | Eğim tespiti, denge kontrolü    |
| Hızlanma                | detect_sign, manage_behavior            | /detect_sign/signs, /get_lidar/scan, /plan_local_path/cmd_vel | Yol açıklığı analizi, hız profili|
| Sudan Geçiş             | detect_sign, manage_behavior            | /detect_sign/signs, /get_imu/data, /plan_local_path/cmd_vel | Su tespiti, özel sürüş stratejisi|
| Trafik Konileri         | detect_obstacle, plan_local_path        | /get_lidar/scan, /get_rgbd_camera/color_image, /plan_local_path/cmd_vel| Koni tespiti, hassas rota planlama|
| Engebeli Arazi/Dik Eğim | get_imu, manage_behavior                | /get_imu/data, /manage_behavior/terrain_status, /plan_local_path/cmd_vel | Arazi analizi, eğim kontrolü    |
| Atış Bölgesi            | detect_object, control_shooting         | /get_targeting_camera/image, /get_rgbd_camera/depth_image, /control_servo/command | Hedef tespiti, lazer hizalama |

---

## ROS2 Temel Kurulum Adımları

### 1. Workspace ve Paket Oluşturma

```bash
# Workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

# Örnek: get_lidar paketi oluştur (Python)
cd src
ros2 pkg create --build-type ament_python get_lidar
# Diğer node'lar için de aynı şekilde paket oluşturun
```

### 2. Custom Mesaj/Servis/Action Tanımları

```bash
# Mesaj paketi oluştur
ros2 pkg create --build-type ament_cmake custom_msgs
ros2 pkg create --build-type ament_cmake custom_srvs
ros2 pkg create --build-type ament_cmake custom_actions

# msg, srv, action dosyalarını ilgili paketlerin 'msg', 'srv', 'action' klasörlerine ekleyin
# Örnek: custom_msgs/msg/SignArray.msg
```

### 3. Node Kodlarını Yazma

Her paketin içinde, örneğin `get_lidar/get_lidar_node.py` gibi dosyalar oluşturun ve ROS2 node kodunuzu yazın.

```python
# get_lidar/get_lidar_node.py (örnek Python node)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('get_lidar')
        self.publisher_ = self.create_publisher(LaserScan, '/get_lidar/scan', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        msg = LaserScan()
        # Dummy data for demonstration
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 0.01
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.12
        msg.range_max = 3.5
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [1.0 for _ in range(num_readings)]
        msg.intensities = [0.0 for _ in range(num_readings)]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 4. Launch Dosyası ile Başlatma

Tüm node'ları başlatmak için bir launch dosyası yazın.

```python
# launch/all_nodes.launch.py (örnek)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='get_lidar', executable='get_lidar_node', name='get_lidar'),
        Node(package='detect_sign', executable='detect_sign_node', name='detect_sign'),
        # ...diğer node'lar...
    ])
```

### 5. Derleme ve Çalıştırma

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch <paket_adı> <launch_dosyası>
# Örnek: ros2 launch get_lidar all_nodes.launch.py
```

---

Bu adımlar, yukarıda tanımladığınız mimarinin temelini ROS2 üzerinde kurmak için yeterlidir. Her node, mesaj, servis ve action için örnek kodları ROS2 belgelerinden veya örneklerinden faydalanarak detaylandırabilirsiniz.
