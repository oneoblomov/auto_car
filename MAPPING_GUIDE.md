# 🗺️ Tank Robot Harita Sistemi Kullanım Rehberi

## 📋 Genel Bakış

Tank robotunuz artık gelişmiş harita özelliklerine sahip! Üç farklı harita türü kullanabilirsiniz:

1. **🏗️ Statik Harita Sunucusu** - Önceden tanımlanmış haritalar
2. **🧭 SLAM (Eş Zamanlı Lokalizasyon ve Haritalama)** - Gerçek zamanlı harita oluşturma
3. **⚡ Dinamik Engel Haritası** - LiDAR tabanlı anlık engel tespiti

## 🚀 Sistemin Başlatılması

### 1. SLAM ile Başlatma (Önerilen)

```bash
cd /home/kaplan/ros2_ws
source install/setup.bash
ros2 launch detect autonomous_driving.launch.py enable_slam:=true enable_map_server:=false
```

### 2. Statik Harita ile Başlatma

```bash
cd /home/kaplan/ros2_ws
source install/setup.bash
ros2 launch detect autonomous_driving.launch.py enable_slam:=false enable_map_server:=true
```

### 3. Sadece Dinamik Engel Tespiti

```bash
cd /home/kaplan/ros2_ws
source install/setup.bash
ros2 launch detect autonomous_driving.launch.py enable_slam:=false enable_map_server:=false
```

## 📊 Görselleştirme (RViz)

Haritaları görselleştirmek için:

```bash
cd /home/kaplan/ros2_ws
source install/setup.bash
ros2 launch detect visualization.launch.py
```

RViz'de görebileceğiniz veriler:

- 🗺️ **Statik Harita** (`/map`) - Mavi tonlarda
- 🧭 **SLAM Haritası** (`/slam_map`) - Yeşil tonlarda
- ⚡ **Engel Haritası** (`/obstacle_map`) - Kırmızı tonlarda
- 📡 **LiDAR Verileri** (`/scan`) - Beyaz noktalar
- 🛤️ **Planlanan Yol** (`/planned_path`) - Yeşil çizgi
- 🤖 **Robot Pozu** (`/slam_pose`) - Kırmızı ok

## 🎮 Harita ile Otonom Sürüş

### Otonom Moda Geçiş

Kontrol arayüzünde `a` tuşuna basın veya:

```bash
ros2 topic pub --once /driving_mode std_msgs/String 'data: "autonomous"'
```

### Hedef Belirleme

Kontrol arayüzünde `g` tuşuna basın ve koordinatları girin, veya:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {w: 1.0}
  }
}'
```

## 🔧 Harita Ayarları

### SLAM Parametreleri

```bash
# Parçacık sayısı (lokalizasyon hassasiyeti)
num_particles: 100

# Harita boyutları
map_width: 20.0   # metre
map_height: 20.0  # metre
resolution: 0.1   # metre/piksel
```

### Statik Harita Parametreleri

```bash
# Harita boyutları
map_width: 20.0
map_height: 20.0
resolution: 0.1

# Engel bölgeleri (x1,y1,x2,y2)
obstacles: [[5,5,7,7], [10,10,12,15]]

# Güvenli koridorlar
corridors: [[0,8,20,12]]
```

## 📡 Önemli Konular (Topics)

### Harita Konuları

- `/map` - Statik harita verisi
- `/slam_map` - SLAM haritası
- `/obstacle_map` - Dinamik engel haritası
- `/slam_pose` - Robot pozisyonu (SLAM)

### Kontrol Konuları

- `/goal_pose` - Hedef pozisyon
- `/driving_mode` - Sürüş modu
- `/cmd_vel` - Hız komutları
- `/autonomous_status` - Otonom durum

### Sensör Konuları

- `/scan` - LiDAR verileri
- `/odom` - Odometri verileri

## 🛠️ Sorun Giderme

### SLAM Çalışmıyor

```bash
# LiDAR verilerini kontrol edin
ros2 topic echo /scan --once

# Odometri verilerini kontrol edin
ros2 topic echo /odom --once

# TF ağacını kontrol edin
ros2 run tf2_tools view_frames
```

### Harita Görünmüyor

```bash
# Harita konularını kontrol edin
ros2 topic list | grep map

# Harita verilerini kontrol edin
ros2 topic echo /slam_map --once
```

### Path Planning Çalışmıyor

```bash
# Hedef pozisyonunu kontrol edin
ros2 topic echo /goal_pose --once

# Otonom durumu kontrol edin
ros2 topic echo /autonomous_status --once
```

## ⚡ Hızlı Komutlar

### Test Senaryosu

```bash
# 1. Sistemi başlat
ros2 launch detect autonomous_driving.launch.py enable_slam:=true

# 2. Otonom moda geç
ros2 topic pub --once /driving_mode std_msgs/String 'data: "autonomous"'

# 3. Hedef belirle
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'

# 4. RViz'de görselleştir
ros2 launch detect visualization.launch.py
```

### Sistemi Durdurma

```bash
# Acil durdurma
ros2 topic pub --once /emergency_stop std_msgs/Bool 'data: true'

# Acil durdurma kaldırma
ros2 topic pub --once /emergency_stop std_msgs/Bool 'data: false'

# Tüm nodeları durdurma
pkill -f ros2
```

## 📈 Performans İpuçları

1. **SLAM İçin**: Yavaş hareket edin, ani dönüşlerden kaçının
2. **Statik Harita İçin**: Önceden haritayı doğru şekilde yapılandırın
3. **Dinamik Engellerden**: LiDAR'ın temiz olduğundan emin olun
4. **RViz Performansı**: Gereksiz görselleştirmeleri kapatın

## 🔄 Sistem Güncelleme

Yeni harita özellikleri ekledikten sonra:

```bash
cd /home/kaplan/ros2_ws
colcon build --packages-select detect
source install/setup.bash
```

---

## 🎯 Sonuç

Artık tank robotunuz gelişmiş harita sistemi ile:

- ✅ Gerçek zamanlı harita oluşturabilir (SLAM)
- ✅ Önceden tanımlanmış haritaları kullanabilir
- ✅ Dinamik engelleri tespit edebilir
- ✅ Akıllı yol planlama yapabilir
- ✅ RViz'de tüm verileri görselleştirebilir

İyi kodlamalar! 🚀🤖
