# 🚗 Tank Robot Otonom Sürüş Sistemi Kullanım Kılavuzu

## 📋 Genel Bakış

Bu sistem, tank tipi robot için kapsamlı bir otonom sürüş çözümü sunar. Motor kontrolünden yol planlamaya, engel algılamadan davranış yönetimine kadar tüm otonom sürüş fonksiyonlarını içerir.

## 🏗️ Sistem Mimarisi

### Node'lar ve Görevleri:

1. **🎯 motor_node** (control paketi)
   - ROS2 komutlarını Unity TCP bağlantısı üzerinden motora iletir
   - `/cmd_vel` topic'ini dinler ve motor hızlarını kontrol eder

2. **👁️ obstacle_node** (detect paketi)
   - LiDAR ve derinlik kamerası ile engel tespit eder
   - Occupancy grid haritası oluşturur
   - `/detect_obstacle/obstacle_map` topic'inde yayınlar

3. **🧠 behavior_manager** (detect paketi)
   - Acil durum, sıkışma, engel kaçınma davranışlarını yönetir
   - Güvenlik kontrollerini sağlar
   - Robot davranış durumunu sürekli izler

4. **🗺️ path_planner** (detect paketi)
   - DWA (Dynamic Window Approach) algoritmasıyla yol planlar
   - Engel haritasını kullanarak optimal rotayı hesaplar
   - Real-time trajectory planlama yapar

5. **🚗 autonomous_driver** (detect paketi)
   - Ana otonom sürüş kontrolcüsü
   - Manuel, otonom, duvar takibi, keşif modları
   - Farklı sürüş senaryolarını koordine eder

6. **🎮 control_interface** (detect paketi)
   - Kullanıcı arayüzü ve kontrol paneli
   - Klavye ile interaktif kontrol
   - Durum izleme ve raporlama

## 🚀 Hızlı Başlangıç

### 1. Sistem Başlatma

```bash
# ROS2 workspace'i source et
cd /home/kaplan/ros2_ws
source install/setup.bash

# Tüm sistemi başlat
ros2 launch detect autonomous_driving.launch.py
```

### 2. Unity Simülasyonu

- Unity projesini açın
- Play butonuna basın
- Robot Unity sahnesinde hazır olacak

### 3. Kontrol Arayüzü

- Launch komutu çalıştırıldıktan sonra terminal'de kontrol arayüzü görünecek
- Klavye komutlarıyla robotu kontrol edebilirsiniz

## 🎮 Kullanım Kılavuzu

### Sürüş Modları:

#### 🟢 Manuel Mod (`m`)

- **w**: İleri git
- **s**: Geri git
- **a**: Sola dön
- **d**: Sağa dön
- **x**: Dur

#### 🤖 Otonom Mod (`a`)

- Robot hedef koordinata otonom olarak gider
- `g` tuşu ile hedef belirleyebilirsiniz
- Format: `5.0,3.0` (x,y koordinatları)

#### 🧱 Duvar Takip Modu (`w`)

- Robot sağ duvara göre takip eder
- Otomatik duvar mesafe kontrolü
- Engellerde otomatik dönüş

#### 🗺️ Keşif Modu (`e`)

- Önceden tanımlı rotada keşif yapar
- Sistematik alan tarama
- Otomatik hedef değiştirme

### Acil Durum Kontrolü:

- **SPACE**: Acil durum toggle (sistem durur)

### Durum Bilgisi:

- **i**: Detaylı sistem durumu
- **h**: Yardım menüsü

## 📊 Sistem Durumu İzleme

### ROS2 Topic'ler:

```bash
# Motor komutları
ros2 topic echo /cmd_vel

# Engel haritası
ros2 topic echo /detect_obstacle/obstacle_map

# Otonom durum
ros2 topic echo /autonomous_status

# Davranış durumu  
ros2 topic echo /behavior_status

# LiDAR verileri
ros2 topic echo /get_lidar/scan
```

### Sistem Logları:
```bash
# Tüm node loglarını görüntüle
ros2 node list
ros2 topic list
```

## ⚙️ Parametre Ayarları

### Motor Kontrolü:

- `max_linear_speed`: 1.5 m/s
- `max_angular_speed`: 1.0 rad/s

### Güvenlik:

- `emergency_distance`: 0.3 m
- `obstacle_distance`: 0.8 m
- `safe_distance`: 1.0 m

### Path Planning:

- `max_speed`: 2.0 m/s
- `predict_time`: 3.0 s
- `robot_radius`: 0.3 m

## 🔧 Sorun Giderme

### Yaygın Problemler:

1. **Unity Bağlantı Sorunu**
   ```bash
   # Motor node loglarını kontrol et
   ros2 topic echo /motor_node/status
   ```

2. **LiDAR Verisi Gelmiyor**
   ```bash
   # LiDAR node'unu kontrol et
   ros2 topic hz /get_lidar/scan
   ```

3. **Robot Hareket Etmiyor**
   ```bash
   # Acil durum durumunu kontrol et
   ros2 topic echo /emergency_stop
   ```

4. **Path Planning Çalışmıyor**
   ```bash
   # Engel haritasını kontrol et
   ros2 topic echo /detect_obstacle/obstacle_map
   ```

### Debug Komutları:
```bash
# Node'ları listele
ros2 node list

# Topic'leri listele  
ros2 topic list

# Belirli bir node'un bilgisini al
ros2 node info /autonomous_driver

# RQT ile görsel debug
rqt_graph
```

## 📈 Performans İpuçları

1. **CPU Optimizasyonu**: 
   - LiDAR işleme frekansını ayarlayın (default: 10Hz)
   - Path planning çözünürlüğünü optimize edin

2. **Bellek Kullanımı**:
   - Occupancy grid boyutunu gereksinime göre ayarlayın
   - Trajectory geçmişi limitini kontrol edin

3. **Network Latency**:
   - Unity TCP bağlantı kalitesini kontrol edin
   - Buffer boyutlarını optimize edin

## 🔄 Sistem Güncelleme

```bash
# Package'ları yeniden build et
cd /home/kaplan/ros2_ws
colcon build --packages-select detect control

# Yeni değişiklikleri source et
source install/setup.bash
```

## 🎯 İleri Seviye Özellikler

### 1. Özel Hedef Belirleme:
```bash
# Komut satırından hedef gönder
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### 2. Manuel Sürüş Modu Değiştirme:
```bash
# Otonom moda geç
ros2 topic pub /driving_mode std_msgs/String "data: 'autonomous'"

# Manuel moda geç  
ros2 topic pub /driving_mode std_msgs/String "data: 'manual'"
```

### 3. Davranış Parametrelerini Değiştirme:

- Node parameter dosyalarını düzenleyerek davranış parametrelerini ayarlayabilirsiniz
- Runtime'da parameter değişiklikleri için `ros2 param set` kullanın

## 📞 Destek

Sorunlar için:

1. Önce bu kılavuzu kontrol edin
2. ROS2 loglarını inceleyin
3. Debug komutlarını kullanın
4. Gerekirse sistem restart yapın

---

🎉 **Başarılı otonom sürüş deneyimi!** 🎉
