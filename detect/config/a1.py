import numpy as np

# === Harita boyutları ve renk kodları ===
width, height = 600, 600
img = np.zeros((height, width), dtype=np.uint8)

# PGM değerleri
ROAD     = 254   # Yol (asfalt)
BARRIER  = 200   # Yol bariyeri
OBSTACLE = 128   # Dik engel, engebeli arazi tümsekleri
GRAVEL   = 100   # Taşlı/çakıllı yol
WATER    = 50    # Sığ su
SLOPE    = 75    # Yan eğim / rampalar
ACCEL    = 180   # Hızlanma bölgesi
CONES    = 25    # Trafik konileri
TARGET   = 220   # Atış hedefi çerçevesi

road_width    = 30    # 3 m = 30 px
barrier_width = 8     # bariyer kalınlığı ~80 cm = 8 px
straight_len  = 200   # 20 m = 200 px

# === Yardımcı fonksiyonlar ===
def draw_road(img, start, end):
    """Kalın yol bandı çizer."""
    x1,y1 = start
    x2,y2 = end
    if x1 == x2:
        ys, ye = sorted([y1,y2])
        img[ys:ye+1, x1-road_width//2 : x1+road_width//2] = ROAD
    else:
        xs, xe = sorted([x1,x2])
        img[y1-road_width//2 : y1+road_width//2, xs:xe+1] = ROAD

def draw_barrier(img, start, end):
    """Yol kenarlarına bariyer çizer."""
    x1,y1 = start
    x2,y2 = end
    if x1 == x2:
        ys, ye = sorted([y1,y2])
        # sol bariyer
        img[ys:ye+1, x1-road_width//2-barrier_width : x1-road_width//2] = BARRIER
        # sağ bariyer
        img[ys:ye+1, x1+road_width//2 : x1+road_width//2+barrier_width] = BARRIER
    else:
        xs, xe = sorted([x1,x2])
        # üst bariyer
        img[y1-road_width//2-barrier_width : y1-road_width//2, xs:xe+1] = BARRIER
        # alt bariyer
        img[y1+road_width//2 : y1+road_width//2+barrier_width, xs:xe+1] = BARRIER

# === 1) Düz 20 m (200px) ===
p0 = (100, 300)
p1 = (100 + straight_len, 300)
draw_road(img, p0, p1)
draw_barrier(img, p0, p1)

# === 2) Viraj: dikey 20 m yukarı ===
p2 = (p1[0], p1[1] - straight_len)
draw_road(img, p1, p2)
draw_barrier(img, p1, p2)

# === 3) Viraj: yatay geri 20 m sola ===
p3 = (p2[0] - straight_len, p2[1])
draw_road(img, p2, p3)
draw_barrier(img, p2, p3)

# === 4) Yan eğim bölgesi (ortalama yol üzeri) ===
#       %20 yan eğim etiketli bölge
# Sadece PGM haritada işaretlemek için yol ortasında SLOPE değeri atıyoruz
sx = (p3[0] + p3[0]) // 2
sy1 = p3[1] - road_width//2
sy2 = p3[1] + road_width//2
img[sy1:sy2, sx - 20 : sx + 20] = SLOPE

# === 5) Hızlanma bölgesi ===
# Öncesinde 20px boşluk, sonra 60px uzunluğunda ACCEL alanı
accel_start = (p3[0] - 60, p3[1])
accel_end   = (p3[0], p3[1])
img[p3[1]-road_width//2 : p3[1]+road_width//2,
    accel_start[0]:accel_end[0]] = ACCEL

# === 6) Sığ su geçişi ===
# Düşey yönde 40px uzunlukta su bandı
w_center_x = accel_start[0] - 40
w_y1 = p3[1] - road_width//2
w_y2 = w_y1 + 40
img[w_y1:w_y2, w_center_x-road_width//2 : w_center_x+road_width//2] = WATER

# === 7) Trafik konileri ===
# Su bitiminden sonraki düz şeritte 5×5 ızgara
cx = w_center_x
cy = w_y2 + 20
for i in range(5):
    for j in range(5):
        if (i+j) % 2 == 0:
            y0 = cy + i* (road_width//2)
            x0 = cx - road_width//2 + j* (road_width//2)
            img[y0:y0+5, x0:x0+5] = CONES

# === 8) Engebeli arazi (tümsek parkuru) ===
# Konilerin sağındaki kısa yatay yol parçasında rastgele tümsekler
bump_y = cy + 5* (road_width//2) + 30
for _ in range(20):
    bx = np.random.randint(p0[0], p1[0])
    by = np.random.randint(bump_y, bump_y + road_width)
    img[by:by+8, bx:bx+8] = OBSTACLE

# === 9) Dik eğim rampası ve atış hedefi ===
# Rampaya yaklaşan dikey yol
r0 = (p1[0], p0[1] + 100)
r1 = (r0[0], r0[1] - 80)  # 80px uzun ramp
draw_road(img, r1, r0)
draw_barrier(img, r1, r0)
# Rampanın üzerine SLOPE değeri
img[r1[1]:r0[1], r0[0]-road_width//2 : r0[0]+road_width//2] = SLOPE
# Hedef çerçevesi (20×30px dikdörtgen)
tx, ty = r1[0] + 20, r1[1] - 40
img[ty:ty+30, tx:tx+20] = TARGET

# === Sonuç: PGM dosyasına yaz ===
with open("/home/kaplan/ros2_ws/src/detect/config/example_map.pgm", "w") as f:
    f.write(f"P2\n{width} {height}\n255\n")
    for row in img:
        f.write(" ".join(map(str, row)) + "\n")
