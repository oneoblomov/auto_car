import numpy as np

width, height = 600, 600
img = np.zeros((height, width), dtype=np.uint8)

# Renk kodları
ROAD = 254        # Beyaz (Yol)
OBSTACLE = 128    # Gri (Genel engel)
GRAVEL = 100      # Koyu gri (Taşlı yol)
WATER = 50        # Mavi tonu (Su)
SLOPE = 75        # Eğim bölgesi
CONES = 25        # Trafik konileri

# Ana yol (3m = 30px ölçekli)
road_width = 30

# Parkur geometrisi (Şekil 2'ye göre)
# Başlangıç -> Dik Engel -> Taşlı Yol -> Yan Eğim -> Hızlanma -> 
# Sığ Su -> Trafik Konileri -> Engebeli Arazi -> Dik Eğim & Atış -> Bitiş

# Ana yol çizgisi
def draw_road(img, start, end, vertical=True):
    if vertical:
        y1, y2 = sorted([start[1], end[1]])
        img[y1:y2+1, start[0]-road_width//2:start[0]+road_width//2] = ROAD
    else:
        x1, x2 = sorted([start[0], end[0]])
        img[start[1]-road_width//2:start[1]+road_width//2, x1:x2+1] = ROAD

# Ana yol segmentleri
draw_road(img, (150, 500), (150, 300), vertical=True)   # Dikey 1
draw_road(img, (150, 300), (450, 300), vertical=False)  # Yatay 1
draw_road(img, (450, 300), (450, 100), vertical=True)   # Dikey 2
draw_road(img, (450, 100), (350, 100), vertical=False)  # Yatay 2

# Engeller
# 1. Dik Engel (20cm yükseklik)
img[280:300, 130:170] = OBSTACLE

# 2. Taşlı/Çakıllı Yol (Segment 2)
img[290:310, 200:400] = GRAVEL

# 3. Yan Eğim (%20 slope alanı)
img[250:270, 50:100] = SLOPE

# 4. Hızlanma Bölgesi (Özel işaretleme)
img[320:340, 400:500] = ROAD  # Temiz yol

# 5. Sığ Su (40cm derinlik)
img[350:370, 220:280] = WATER

# 6. Trafik Konileri (5x5 grid)
for i in range(5):
    for j in range(5):
        if (i+j)%2 == 0:
            img[400+i*10:403+i*10, 300+j*10:303+j*10] = CONES

# 7. Engebeli Arazi (Rastgele tümsekler)
for _ in range(20):
    x = np.random.randint(100, 500)
    y = np.random.randint(100, 200)
    img[y:y+8, x:x+8] = OBSTACLE

# 8. Dik Eğim & Atış Bölgesi
img[50:70, 320:380] = SLOPE
img[30:40, 350:360] = 200  # Hedef alanı

# Dosyaya yazma
with open("example_map.pgm", "w") as f:
    f.write("P2\n{} {}\n255\n".format(width, height))
    for row in img:
        f.write(" ".join(map(str, row)) + "\n")