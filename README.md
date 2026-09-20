# SBLTR

Firmware robot self-balancing ESP32

## Struktur kode

| Berkas | Tanggung jawab |
| --- | --- |
| `src/main.cpp` | Memulai modul dan menjalankan loop utama. |
| `src/RobotController.cpp` | State machine, PID balancing, dan penanganan robot jatuh. |
| `src/Imu.cpp` | MPU6050, kalibrasi, sudut roll, dan complementary filter. |
| `src/Motors.cpp` | Arah motor, PWM, slew limit, dan konversi keluaran PID. |
| `src/LineFollower.cpp` | Pembacaan lima sensor garis dan koreksi belok. |
| `src/WebControl.cpp` | Wi-Fi, halaman kontrol, dan endpoint `/set` serta `/drive`. |
| `src/SerialTuning.cpp` | Perintah tuning melalui Serial. |
| `include/HardwareConfig.h` | Pin dan konstanta perangkat keras. |
| `include/RobotContext.h` | Parameter dan status yang dipakai bersama. |

Header untuk setiap modul berada di `include/`. Nilai awal pin, PID, sensor garis, dan logika kontrol mengikuti sketch asal. Modul motor memakai API PWM yang sesuai untuk Arduino ESP32 2.x atau 3.x.

## Menjalankan

1. Pastikan `include/WifiCredentials.h` tersedia. Jika belum, salin `include/WifiCredentials.example.h` ke nama tersebut dan isi SSID serta kata sandi. Berkas kredensial lokal dikecualikan oleh `.gitignore`.
2. Jalankan `pio run -e esp32dev` untuk build.
3. Jalankan `pio run -e esp32dev -t upload` untuk unggah ke ESP32. Buka monitor Serial pada 115200 baud.

Setelah terhubung ke Wi-Fi, alamat IP robot dicetak pada Serial. Buka alamat tersebut untuk mengubah PID dan memberi perintah arah.

## Catatan dari sketch asal

- Sensor garis aktif secara bawaan dan dapat mengubah setpoint pada setiap siklus balancing. Karena itu, perintah maju/mundur dari web dapat tertimpa pada siklus berikutnya.
- `MPU6050_light::update()` tidak menyediakan status kegagalan pembacaan; pemeriksaan `readImu()` masih mengikuti perilaku sketch asal. Keberadaan MPU diperiksa saat inisialisasi.
- Koneksi Wi-Fi pada `setup()` menunggu hingga berhasil sebelum kontrol robot mulai berjalan.
