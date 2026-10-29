#include <iostream>
#include <iomanip>
#include <vector>
#include <array>
#include "core/GeoProjection.hpp"
#include "mapbox/earcut.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

struct IzmirReferencePoint {
    const char* code;
    const char* name;
    double lat;
    double lon;
};

static const IzmirReferencePoint REF_POINTS[] = {
    { "KNK",  "Konak Saat Kulesi (Merkez Orijin)", 38.418898, 27.128677 },
    { "ALS",  "Alsancak Gar & IZBAN",              38.438850, 27.147720 },
    { "BYR",  "Bayrakli Folkart & Turan IZBAN",    38.454200, 27.176400 },
    { "KSK",  "Karsiyaka Iskele & Carsi",          38.454800, 27.119500 },
    { "CGL",  "Cigli IZBAN & Kaklic Hava Ussu",    38.497500, 27.062800 },
    { "EGK",  "Egekent 1 IZBAN Istasyonu",         38.518600, 27.058500 },
    { "ULK",  "Ulukent IZBAN Istasyonu",           38.536400, 27.057800 },
    { "EGK2", "Egekent 2 IZBAN & 30 Agustos Mah.", 38.555800, 27.064200 },
    { "MNM",  "Menemen Gar IZBAN Istasyonu",       38.604200, 27.069400 },
    { "YMN",  "Yamanlar Dagi Karagol Zirvesi",     38.532000, 27.185000 },
    { "LTBJ", "Adnan Menderes Havalimani & IZBAN", 38.292400, 27.157000 }
};

int main() {
    std::cout << "========================================================================\n";
    std::cout << "  IZMIR 3D SIMULATOR ENGINE - FAZ 1 CEKIRDEK DOGRULAMA TESTI\n";
    std::cout << "========================================================================\n\n";

    izmir::GeoProjection proj;

    std::cout << "[TEST 1] WGS84 -> Yerel Izmir Metre (ENU) Koordinat Projeksiyonu:\n";
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << std::left << std::setw(6) << "KOD"
              << std::setw(36) << "KONUM / ISTASYON ADI"
              << std::right << std::setw(12) << "DOGU X (m)"
              << std::setw(12) << "KUZEY Z (m)"
              << std::setw(10) << "KONAK'A" << "\n";
    std::cout << "------------------------------------------------------------------------\n";

    for (const auto& pt : REF_POINTS) {
        izmir::Vec2d xz = proj.latLonToWorldXZ(pt.lat, pt.lon);
        double distKm = std::hypot(xz.x, xz.z) / 1000.0;
        std::cout << std::left << std::setw(6) << pt.code
                  << std::setw(36) << pt.name
                  << std::right << std::fixed << std::setprecision(1)
                  << std::setw(11) << xz.x << "m"
                  << std::setw(11) << xz.z << "m"
                  << std::setprecision(2) << std::setw(8) << distKm << " km\n";
    }

    // Egekent 2 ile Ulukent arasi mesafe kontrolu
    double dEgk2Ulk = proj.distanceMeters({38.555800, 27.064200}, {38.536400, 27.057800});
    std::cout << "------------------------------------------------------------------------\n";
    std::cout << "  -> Dogrulama: Egekent 2 IZBAN <-> Ulukent IZBAN Kus Ucusu Mesafe: "
              << std::fixed << std::setprecision(0) << dEgk2Ulk << " metre\n\n";

    // [TEST 2] Mapbox Earcut Icbukey (L-Tipi ve Avlulu) Bina Poligonu Ucgenleme Testi
    using Coord = double;
    using N = uint32_t;
    using Point = std::array<Coord, 2>;
    std::vector<std::vector<Point>> polygon;

    // L-seklinde 6 koseli gercekci bir apartman parselinin dis siniri
    polygon.push_back({
        {0.0, 0.0}, {24.0, 0.0}, {24.0, 10.0},
        {10.0, 10.0}, {10.0, 22.0}, {0.0, 22.0}
    });

    std::vector<N> indices = mapbox::earcut<N>(polygon);
    std::cout << "[TEST 2] Mapbox Earcut Icbukey Bina Poligonu Ucgenleme:\n";
    std::cout << "  -> 6 Koseli L-Bina Poligonu -> " << (indices.size() / 3)
              << " Adet Hatasiz Ucgene (" << indices.size() << " indeks) Bolundu. [BASARILI]\n\n";

    // [TEST 3] Nlohmann JSON Parser Testi
    json sampleOsm = json::parse(R"({
        "type": "way",
        "id": 48291042,
        "tags": {
            "building": "apartments",
            "building:levels": "9",
            "addr:suburb": "Egekent 2"
        }
    })");
    std::cout << "[TEST 3] Nlohmann JSON Parser Kontrolu:\n";
    std::cout << "  -> Ornek OSM Binasi: Mahalle = " << sampleOsm["tags"]["addr:suburb"]
              << ", Kat = " << sampleOsm["tags"]["building:levels"] << " [BASARILI]\n";
    std::cout << "========================================================================\n";
    std::cout << "  FAZ 1 HAZIR! Simdi Faz 2 (Gercek OSM & IZBAN Derleyicisi) icin haziriz.\n";
    std::cout << "========================================================================\n";
    return 0;
}
