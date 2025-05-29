# 🤖 DETECT Paketi Kullanım Kılavuzu

## 📋 İçindekiler

- [Genel Bakış](#genel-bakış)
- [Sistem Mimarisi](#sistem-mimarisi)
- [Node'lar ve Görevleri](#nodelar-ve-görevleri)
- [Hızlı Başlangıç](#hızlı-başlangıç)
- [Sürüş Modları](#sürüş-modları)
- [Topic'ler ve Mesajlar](#topicler-ve-mesajlar)
- [Konfigürasyon](#konfigürasyon)
- [Sorun Giderme](#sorun-giderme)
- [İleri Seviye Kullanım](#ileri-seviye-kullanım)

---

## 🎯 Genel Bakış

**Detect** paketi, tank robotunun otonom sürüş sisteminin beyni görevi görür. Bu paket şu ana bileşenleri içerir:

- **Engel Algılama**: LiDAR ve kamera ile çevre algısı
- **Yol Planlama**: DWA algoritmasıyla optimal rota hesaplama
- **Otonom Sürüş**: Çoklu mod destekli akıllı sürüş kontrolü
- **Davranış Yönetimi**: Güvenlik ve acil durum kontrolü
- **Kontrol Arayüzü**: Kullanıcı etkileşimi ve izleme

---

## 🏗️ Sistem Mimarisi

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Unity Sim     │───▶│   GET Package    │───▶│  DETECT Package │
│  (Sensörler)    │    │   (Veri Alma)    │    │  (Algı & Karar) │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
                                                         ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│     Robotun     │◀───│ CONTROL Package  │◀───│   Motor Cmd     │
│   Fiziksel      │    │  (Motor Kontrol) │    │   (/cmd_vel)    │
│   Hareket       │    └──────────────────┘    └─────────────────┘
└─────────────────┘
```

---

## 🤖 Node'lar ve Görevleri

### 1. 👁️ **obstacle_node.py**

**Görev**: Engel algılama ve harita oluşturma

- LiDAR verilerini işler
- Derinlik kamerası verilerini analiz eder
- OccupancyGrid haritası oluşturur
- Real-time engel tespiti yapar

**Ana Topic'ler**:

- **Girdi**: `/get_lidar/scan`, `/get_rgbd_camera/depth_image`
- **Çıktı**: `/detect_obstacle/obstacle_map`

### 2. 🗺️ **path_planner_node.py**

**Görev**: Yol planlama ve trajectory hesaplama

- DWA (Dynamic Window Approach) algoritması
- Statik, SLAM ve dinamik haritaları birleştirir
- Optimal hız ve dönüş komutları hesaplar
- Engel kaçınma stratejileri

**Ana Topic'ler**:

- **Girdi**: `/detect_obstacle/obstacle_map`, `/goal_pose`, `/map`
- **Çıktı**: `/cmd_vel`, `/planned_path`

### 3. 🚗 **autonomous_driver_node.py**

**Görev**: Ana otonom sürüş kontrolcüsü

- 4 farklı sürüş modu yönetimi
- Hedef belirleme ve takip
- Acil durum koordinasyonu
- Sistem durumu izleme

**Ana Topic'ler**:

- **Girdi**: `/driving_mode`, `/goal_pose`, `/emergency_stop`
- **Çıktı**: `/cmd_vel`, `/autonomous_status`

### 4. 🧠 **behavior_manager_node.py**

**Görev**: Davranış koordinasyonu ve güvenlik

- Acil durum tespiti
- Sıkışma algılaması
- Güvenlik protokolleri
- Sistem sağlığı izleme

### 5. 🎮 **control_interface_node.py**

**Görev**: Kullanıcı arayüzü ve kontrol paneli

- Klavye kontrolleri
- Mod değiştirme
- Sistem durumu gösterimi
- Real-time parametre ayarları

---

## 🚀 Hızlı Başlangıç

### 1. Sistem Başlatma

```bash
# ROS2 workspace'i hazırla
cd /home/kaplan/ros2_ws
source install/setup.bash

# Tüm detect sistemini başlat
ros2 launch detect autonomous_driving.launch.py
```

### 2. Unity Simülasyonunu Başlat

- Unity projesini aç
- Play butonuna bas
- Robot Unity sahnesinde aktif olacak

### 3. Otonom Modu Aktifleştir

```bash
# Otonom mod açma
ros2 topic pub --once /driving_mode std_msgs/String 'data: "autonomous"'

# Hedef gönderme
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"}, 
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0}, 
    orientation: {w: 1.0}
  }
}'
```

---

## 🎮 Sürüş Modları

### 🟢 1. Manuel Mod (`manual`)

```bash
ros2 topic pub --once /driving_mode std_msgs/String 'data: "manual"'
```

- Kullanıcı kontrolü
- Klavye ile yönlendirme
- Güvenlik kontrolü aktif

### 🤖 2. Otonom Mod (`autonomous`)

```bash
ros2 topic pub --once /driving_mode std_msgs/String 'data: "autonomous"'
```

- Hedef nokta takibi
- Otomatik engel kaçınma
- DWA yol planlama

### 🧱 3. Duvar Takip Modu (`follow_wall`)

```bash
ros2 topic pub --once /driving_mode std_msgs/String 'data: "follow_wall"'
```

- Sağ duvara göre ilerleme
- Sabit mesafe korunması
- Köşe takibi

### 🗺️ 4. Keşif Modu (`explore`)

```bash
ros2 topic pub --once /driving_mode std_msgs/String 'data: "explore"'
```

- Otomatik alan keşfi
- Sistematik tarama
- Hedef rotasyon

---

## 📡 Topic'ler ve Mesajlar

### 📥 Giriş Topic'leri

| Topic | Mesaj Tipi | Açıklama |
|-------|------------|----------|
| `/get_lidar/scan` | `sensor_msgs/LaserScan` | LiDAR verisi |
| `/get_rgbd_camera/depth_image` | `sensor_msgs/Image` | Derinlik kamerası |
| `/get_encoder/odom` | `nav_msgs/Odometry` | Robot pozisyonu |
| `/driving_mode` | `std_msgs/String` | Sürüş modu |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Hedef pozisyon |
| `/emergency_stop` | `std_msgs/Bool` | Acil durdurma |

### 📤 Çıkış Topic'leri

| Topic | Mesaj Tipi | Açıklama |
|-------|------------|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Motor komutları |
| `/detect_obstacle/obstacle_map` | `nav_msgs/OccupancyGrid` | Engel haritası |
| `/planned_path` | `nav_msgs/Path` | Planlanan yol |
| `/autonomous_status` | `std_msgs/String` | Sistem durumu |
| `/behavior_status` | `std_msgs/String` | Davranış durumu |

---

## ⚙️ Konfigürasyon

### DWA Path Planner Parametreleri

```python
# Hız limitleri
max_speed = 2.0        # m/s
min_speed = -1.0       # m/s
max_yaw_rate = 40°     # derece/s

# Güvenlik parametreleri
robot_radius = 0.3     # metre
safe_distance = 1.0    # metre
predict_time = 3.0     # saniye
```

### Obstacle Detection Parametreleri

```python
# Harita ayarları
map_width = 200        # piksel
map_height = 200       # piksel
map_resolution = 0.1   # metre/piksel

# Algılama mesafeleri
max_lidar_range = 10.0 # metre
obstacle_threshold = 0.3 # metre
```

### Autonomous Driver Parametreleri

```python
# Hız kontrolü
max_linear_speed = 1.5  # m/s
max_angular_speed = 1.0 # rad/s

# Hedef toleransları
goal_tolerance = 0.5    # metre
angle_tolerance = 0.1   # radyan
```

---

## 🔧 Sorun Giderme

### ❌ Yaygın Problemler

#### 1. **Robot hareket etmiyor**

```bash
# Acil durum kontrol et
ros2 topic echo /emergency_stop --once

# Motor komutlarını kontrol et
ros2 topic echo /cmd_vel

# Çözüm: Acil durumu kaldır
ros2 topic pub --once /emergency_stop std_msgs/Bool 'data: false'
```

#### 2. **LiDAR verisi gelmiyor**

```bash
# LiDAR topic'ini kontrol et
ros2 topic hz /get_lidar/scan

# Node durumunu kontrol et
ros2 node list | grep lidar
```

#### 3. **Path planning çalışmıyor**

```bash
# Engel haritasını kontrol et
ros2 topic echo /detect_obstacle/obstacle_map --once

# Hedef kontrol et
ros2 topic echo /goal_pose --once
```

#### 4. **Unity bağlantı sorunu**

```bash
# Motor node loglarını kontrol et
ros2 node info /motor_control

# TCP bağlantısını kontrol et
netstat -an | grep 25001
```

### 🛠️ Debug Komutları

#### Sistem durumu kontrolü

```bash
# Tüm node'ları listele
ros2 node list

# Topic frekanslarını kontrol et
ros2 topic hz /cmd_vel
ros2 topic hz /get_lidar/scan
ros2 topic hz /autonomous_status

# Node bilgilerini görüntüle
ros2 node info /autonomous_driver
ros2 node info /path_planner
ros2 node info /obstacle_detection
```

#### Gerçek zamanlı izleme

```bash
# Sistem durumunu izle
watch -n 1 "ros2 topic echo /autonomous_status --once"

# Engel haritasını izle
ros2 topic echo /detect_obstacle/obstacle_map

# Motor komutlarını izle
ros2 topic echo /cmd_vel
```

---

## 🎯 İleri Seviye Kullanım

### 1. **Özel Hedef Belirleme**

```bash
# Python script ile hedef gönderme
python3 << EOF
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = Node('goal_sender')
pub = node.create_publisher(PoseStamped, '/goal_pose', 10)

goal = PoseStamped()
goal.header.frame_id = 'map'
goal.header.stamp = node.get_clock().now().to_msg()
goal.pose.position.x = 10.0
goal.pose.position.y = 5.0
goal.pose.orientation.w = 1.0

pub.publish(goal)
print("Hedef gönderildi: (10, 5)")
EOF
```

### 2. **Dinamik Parametre Değişimi**

```bash
# Hız limitlerini değiştir
ros2 param set /path_planner max_speed 1.0
ros2 param set /autonomous_driver max_linear_speed 0.8

# Güvenlik mesafesini ayarla
ros2 param set /path_planner robot_radius 0.4
```

### 3. **Özel Davranış Ekleme**

`autonomous_driver_node.py` dosyasına yeni mod eklemek:

```python
class DrivingMode(Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    FOLLOW_WALL = "follow_wall"
    EXPLORE = "explore"
    CUSTOM_MODE = "custom"  # Yeni mod

def handle_custom_mode(self):
    """Özel davranış implementasyonu"""
    cmd_vel = Twist()
    # Özel algoritmanız buraya
    self.cmd_vel_pub.publish(cmd_vel)
```

### 4. **Çoklu Robot Desteği**

```bash
# Robot namespace ile başlatma
ros2 launch detect autonomous_driving.launch.py robot_namespace:=robot1

# İkinci robot
ros2 launch detect autonomous_driving.launch.py robot_namespace:=robot2
```

---

## 📊 Performans Optimizasyonu

### 1. **CPU Optimizasyonu**

```python
# Timer frekanslarını ayarla
self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
self.map_timer = self.create_timer(0.2, self.publish_map)  # 5 Hz
```

### 2. **Bellek Yönetimi**

```python
# Harita boyutunu optimize et
map_width = 100   # Küçült
map_height = 100  # Küçült
resolution = 0.2  # Çözünürlüğü azalt
```

### 3. **Network Optimizasyonu**

```python
# Queue boyutlarını ayarla
self.publisher = self.create_publisher(msg_type, topic, 5)  # Küçük queue
```

---

## 📈 Monitoring ve Analiz

### 1. **Sistem Metrikleri**

```bash
# Resource kullanımı
htop
ros2 wtf  # ROS system durumu

# Topic latency ölçümü
ros2 topic delay /cmd_vel /get_lidar/scan
```

### 2. **Log Analizi**

```bash
# Node loglarını görüntüle
ros2 log view /autonomous_driver
ros2 log view /path_planner

# Log seviyesi değiştir
ros2 param set /autonomous_driver log_level DEBUG
```

### 3. **Performans Grafikleri**

```bash
# rqt ile görselleştirme
rqt_graph
rqt_plot /cmd_vel/linear/x
rqt_console
```

---

## 🔄 Sistemin Güncellenmesi

### 1. **Package Build**

```bash
cd /home/kaplan/ros2_ws
colcon build --packages-select detect
source install/setup.bash
```

### 2. **Konfigürasyon Güncellemeleri**

```bash
# Yeni parametre eklemek için package.xml güncelle
# setup.py'da entry points güncelle
# Launch dosyalarını düzenle
```

### 3. **Test Prosedürü**

```bash
# Birim testler
colcon test --packages-select detect

# Integration test
ros2 launch detect autonomous_driving.launch.py --test

# Performans testi
ros2 run detect performance_test
```

---

## 📞 Destek ve Geliştirme

### Geliştirici Bilgileri

- **Paket**: detect
- **Versiyon**: 0.0.0
- **Lisans**: MIT
- **Geliştirici**: kaplan (<222802078@ogr.cbu.edu.tr>)

### Katkıda Bulunma

1. Repository'yi fork edin
2. Feature branch oluşturun
3. Değişikliklerinizi test edin
4. Pull request gönderin

### Bug Raporlama

- GitHub Issues kullanın
- Detaylı log çıktıları ekleyin
- Sistem konfigürasyonunu belirtin

---

## 🎉 Sonuç

Bu kılavuz ile Detect paketinin tüm özelliklerini kullanabilir, sistemi optimize edebilir ve sorunları çözebilirsiniz. Daha fazla bilgi için kod içindeki yorumları ve ROS2 dokümantasyonunu inceleyebilirsiniz.
**İyi otonom sürüşler! 🚗🤖**
