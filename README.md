# ✈️ İzmir 3D Flight & Urban GIS Simulator (`izmir-3d-flight-sim`)

C++17, OpenGL/GLSL ve X11 kullanılarak sıfırdan geliştirilmiş; **Overture Maps (Microsoft AI + Google Open Buildings)**, **OpenStreetMap** ve **SRTM 3D Topografya** verilerini gerçek zamanlı işleyen İzmir 3D Uçuş ve Şehir Simülatörü.

## 🚀 Öne Çıkan Özellikler

* **Gerçek Coğrafi & Uydu Verisi (`izmir_gis_compiler`):**
  * **Overture Maps & OSM Entegrasyonu:** Konak, Alsancak, Karşıyaka, Bayraklı, Bornova, Çiğli, Mavişehir, Menemen, Egekent 2 ve Ulukent dahil tüm İzmir'in gerçek bina poligonları (`mapbox::earcut` üçgenleme) ve ulaşım ağı.
  * **WGS84 Elipsoid Projeksiyonu:** Konak Saat Kulesi (`38.418898° N, 27.128677° E`) merkezli milimetrik yerel metre (`ENU`) dönüşümü.
  * **SRTM DEM & Kıyı Maskesi:** Flood-Fill algoritmasıyla temizlenmiş gerçek İzmir Körfezi kıyı şeridi ve Yamanlar / Çatalkaya / Nif dağı topografyası.
* **Özel GLSL Shader & 3-Kademeli LOD Motoru (60+ FPS):**
  * `16 x 24` bölgesel donanım VBO kümeleme, Frustum Culling ve 3 kademeli bina detay seviyesi (`vboHigh`, `vboMid`, `vboLow`).
  * `2048 x 2048` vektör yol/parsel arazi dokusu (`uRoadMapTex`) + 3D şeritli otoyol, bulvar, alt geçit ve kavşak geometrisi.
  * ACES Filmic Tone Mapping, Mie atmosferik güneş saçılımı, gece modunda 2.600 titreşen yıldız, 3D Dolunay ve sokak lambası ışık havuzları.
* **Canlı İzmir Dünyası & Fizik:**
  * **İZBAN Ağı:** 29 gerçek İZBAN istasyonu (üstlerinde dönen 3D yıldız işaretçileriyle) ve Menemen $\leftrightarrow$ Adnan Menderes Havalimanı (`LTBJ`) arasında kesintisiz ray grafiği üzerinde hareket eden 3 vagonlu 3D İZBAN tren setleri.
  * **Körfez & Trafik:** Konak–Karşıyaka–Bostanlı–Alsancak arasında çalışan 3D İZDENİZ vapurları ve otoyol/bulvar araç trafiği.
  * **3D Bina Çarpışma Motoru:** `128 x 192` Uzamsal Hash (Spatial Grid) tabanlı gerçek poligon çarpışma testi ve 3D kaza/patlama fiziği.
  * **Kontrolcü Desteği:** Linux `/dev/input/js0` üzerinden tak-çıkar (hot-plug) gamepad/direksiyon desteği ve klavye hibrit kontrolü.

---

## 🎮 Kontroller

| Tuş / Kontrolcü | İşlev |
| :--- | :--- |
| **`W / S` veya `Yukarı / Aşağı`** | Burun Aşağı / Yukarı (Pitch) veya Drone İleri / Geri |
| **`A / D` veya `Sol / Sağ`** | Yatış (Roll) veya Drone Dönüş |
| **`Q / E`** | Dümen (Yaw) veya Drone İrtifa |
| **`Shift / Ctrl`** | Gaz Artır / Azalt (`Throttle %`) |
| **`[F]`** | **64-Bit Uçak Fiziği** $\leftrightarrow$ **Serbest Drone Kamera** Geçişi |
| **`[V]` / `[C]`** | Uçak Modeli Değiştir (Cessna 172, Jet, Planör) / Kamera Açısı Değiştir |
| **`[T]` / `[G]`** | Gece Modu (Yıldızlar & 3D Ay) / Altın Gün Batımı Modu (Golden Hour) |
| **`[Z]`** | Vektör GPS & İZBAN Radar Yakın / Geniş Ölçek |
| **`[1] .. [0]`** | İzmir Bölgeleri Arasında Anında Işınlanma (Konak, Alsancak, Bayraklı, Karşıyaka, Egekent 2, Ulukent, Menemen, Çiğli, Mavişehir, LTBJ) |
| **`[R]` veya `[Space]`** | Bina/Arazi Kazası Sonrası Yeniden Uçuşa Başla |

---

## 🛠️ Kurulum ve Çalıştırma (Linux)

```bash
# 1. Gerekli sistem paketleri (Fedora / Arch / Ubuntu)
# Fedora: sudo dnf install cmake gcc-c++ libX11-devel mesa-libGL-devel libcurl-devel libpng-devel python3-virtualenv

# 2. Overture Maps verisini indir ve harita paketini (.izmb) derle
mkdir -p data/overture data/raw_osm data/compiled build
python3 -m venv .venv
./.venv/bin/pip install overturemaps
./.venv/bin/overturemaps download --bbox=27.00,38.28,27.24,38.62 -f geojsonseq --type=building -o data/overture/izmir_buildings.geojsonseq
./.venv/bin/overturemaps download --bbox=27.00,38.28,27.24,38.62 -f geojsonseq --type=segment -o data/overture/izmir_roads.geojsonseq

# 3. C++ Derleyicisi ve Simülatörü Çalıştır
cd build
cmake ..
make -j$(nproc)
./izmir_gis_compiler
./izmir_sim_phase4
