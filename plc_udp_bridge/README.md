# PLC UDP bridge ve görev adaptörü

Bu ROS 2 Humble paketi iki düğüm içerir:

- `plc_udp_bridge_node`: PLC ile çift yönlü UDP haberleşmesi.
- `plc_mission_adapter_node`: PLC'nin 3 byte görevini MissionManager
  komutlarına dönüştüren, durum onaylı görev makinesi.

## Protokol

Robot saniyede bir kez 7 byte gönderir:

```text
Byte0 status | Byte1 pickup | Byte2 dropoff | Byte3-4 X | Byte5-6 Y
```

Paket biçimi `<BBBhh>` little-endian'dır. X/Y fiziksel olarak metredir;
pakette `integer(metre * 100)` signed Int16 değeri taşınır.

PLC her geçerli robot paketine 3 byte cevap verir:

```text
Byte0 pickup | Byte1 dropoff | Byte2 control
control=1 Bekle, control=2 Başla/Devam et
```

## Derleme

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select plc_udp_bridge --symlink-install
source install/setup.bash
```

## Çalıştırma

Yarışma PLC adresi varsayılan olarak `192.168.100.100` değeridir. DHCP ile
test yaparken PLC bilgisayarının güncel adresini launch argümanıyla verin:

```bash
ros2 launch plc_udp_bridge plc_udp_bridge.launch.py \
  plc_ip:=192.168.1.52
```

Bu komut bridge ve adapter düğümlerini birlikte başlatır. Yalnız ham UDP testi:

```bash
ros2 launch plc_udp_bridge plc_udp_bridge.launch.py \
  plc_ip:=192.168.1.52 start_adapter:=false
```

`local_ip` varsayılanı `0.0.0.0` olduğu için Jetson IP'si ağ değişiminde
YAML içinde değiştirilmek zorunda değildir.

## PLC simülatörü

`plc_simulator.py` dosyası ROS gerektirmez. PLC bilgisayarında:

```bash
python3 plc_simulator.py \
  --bind-ip 0.0.0.0 \
  --port 1515 \
  --pickup 1 \
  --dropoff 2 \
  --control 2
```

Simülatör varsayılan olarak robot paketinde Byte0=`5` görünce otomatik kapı
çevrimi başlatır: hemen control=`1`, 5 saniye sonra control=`2` gönderir.
Süre `--door-delay 3` gibi değiştirilebilir. Elle kontrol için
`--no-auto-door` kullanılabilir.

Çalışırken kullanılabilecek komutlar:

```text
show
set 1 2 1
set 1 2 2
help
quit
```

## Görev akışı

PLC `[1, 2, 2]` gönderdiğinde adapter:

1. `CREATE A1 B2` gönderir ve mission state `1` onayını bekler.
2. `APPROVE` gönderir ve mission state `2` onayını bekler.
3. Control `2` ise `START` gönderir ve mission state `3` onayını bekler.

Control `1` ise görev oluşturulup onaylanır fakat `START` gönderilmez. Çalışan
görevde control `1` PAUSE, tekrar control `2` RESUME üretir. Her komut 2 saniye
içinde onaylanmazsa en fazla 3 kez yeniden gönderilir; ardından PLC durum `7`
yayınlanır ve görev iptal edilir.

PLC paketinde görev sıra numarası olmadığı için tamamlanan A1-B2 görevinin
aynı `[1,2,2]` paketiyle sonsuza kadar yeniden başlaması engellenmiştir. Aynı
görevi tekrar çalıştırmak için PLC önce `[1,2,1]`, sonra `[1,2,2]` göndermelidir.
Farklı bir görev terminal durumdan sonra doğrudan kabul edilir.

## K1/K2 kapı bekleme akışı

Mission Controller, graph düğümünün `arrivalAction` alanındaki `DoorK1` veya
`DoorK2` değerini kapı olarak yorumlar. Düğüm adı `N1`, `N2` olarak kalır;
QR kapı işaretleri bu davranıştan bağımsızdır.

İlgili kapıya gelindiğinde Nav2'ye yeni hedef gönderilmez ve mission status
içindeki `automationWaiting` alanı `true` olur. Adapter bu sırada PLC'ye
Byte0=`5` gönderir. Devam için kapı beklemesi başladıktan sonra PLC'den önce
control=`1`, ardından control=`2` görülmesi zorunludur. Eski veya erken bir
control=`2` robotu hareket ettirmez. Geçerli `1 -> 2` geçişinde adapter
`/amr/automation_continue` topicine Bool `true` gönderir ve rota kaldığı
yerden devam eder.

Bu bekleme mission `PAUSE/RESUME` değildir. Görev `RUNNING` kalır; yalnızca
sıradaki navigasyon hedefi tutulur. PLC bağlantısı kesilir veya control=`2`
gelmezse robot kapıda beklemeye devam eder. Hata `7` ve acil stop `8`, kapı
bekleme durumu `5` değerinden önceliklidir.

## PLC durum kodları

```text
1 Göreve hazır
2 Görev alındı/işleniyor
3 Yüksüz hareket
4 Yüklü hareket
5 Otomasyon komutu bekleniyor/duraklatıldı
6 Görev tamamlandı
7 Hata
8 Acil stop
```

Mission stage 1-3 durum `3`, stage 4-6 durum `4` ve START noktasına
dönüş aşaması olan stage 7 durum `6` olarak PLC'ye çevrilir.
`/amr/emergency_stop` Bool true olduğunda adapter `CANCEL` gönderir ve durum
`8` yayınlar.

## Kontrol topicleri

```bash
ros2 topic echo /plc/mission_command
ros2 topic echo /plc/connected --qos-durability transient_local
ros2 topic echo /plc/adapter_state --qos-durability transient_local
ros2 topic echo /amr/mission_command
ros2 topic echo /amr/mission_status_json --qos-durability transient_local
ros2 topic echo /amr/automation_continue
```

Bridge'in PLC'ye göndereceği alanlar:

```text
/plc/tx_status
/plc/tx_pickup
/plc/tx_dropoff
```

Bu üç publisher ve bridge subscriber'ları transient-local olduğundan düğüm
başlatma sırası son değerlerin kaybolmasına neden olmaz.

## Simülasyon testi

1. `graph_receiver_node` çalıştırın.
2. PLC bilgisayarında simülatörü control `2` ile çalıştırın.
3. Jetson'da bu paketin launch dosyasını doğru `plc_ip` ile başlatın.
4. `/amr/mission_status_json` içinde A1/B2 ve sırasıyla state 1, 2, 3
   değişimlerini kontrol edin.

Nav2/TF yoksa START sonrasında mission state `7` görülebilir. Bu UDP veya
adapter hatası değildir; görev yöneticisinin navigasyon başlatamadığını gösterir.
Görev akışını hareketsiz test etmek için simülatörde control `1` kullanılabilir.

Şartnamedeki durum `6`, yük bırakıldıktan sonra başlangıç noktasına dönüşü
ifade eder. `amr_mission_controller`, `LIFT_DOWN_COMPLETE` sonrasında START
istasyonuna gider ve START'a ulaşınca görevi tamamlar. START istasyonunun
`qr_stations.json` içinde bulunması ve geçerli bir graph noktasına bağlı olması
zorunludur.
