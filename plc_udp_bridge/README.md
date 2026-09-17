# PLC UDP katmanı

Bu launch yalnızca UDP bridge düğümünü başlatır; mission adapter otomatik başlamaz.
Robot IP `172.20.10.2`, PLC bilgisayarı `172.20.10.6`, PLC portu UDP `1515`.
Jetson `local_port=0` ile işletim sisteminin seçtiği kaynak portunu kullanır.
Simülatör cevabı alınan paketin kaynak IP ve portuna gönderir.

## PLC bilgisayarı

`plc_simulator.py` dosyasını PLC bilgisayarına kopyalayıp bulunduğu dizinde:

```bash
python3 plc_simulator.py --bind-ip 0.0.0.0 --port 1515 --pickup 1 --dropoff 2 --control 2
```

ROS gerektirmez; yalnızca Python standart kütüphanesini kullanır.
Terminal komutları:

```text
show
set 1 2 1
set 1 2 2
help
quit
```

Yeni değerler bir sonraki geçerli robot paketinin cevabında gönderilir.
Ctrl+C veya `quit` socket'i kapatır. Robot paketi alınmadan cevap gönderilmez.

## Jetson derleme ve çalıştırma

Paketin `~/ros2_ws/src/plc_udp_bridge` altında bulunduğu varsayılır:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select plc_udp_bridge --symlink-install
source install/setup.bash
ros2 launch plc_udp_bridge plc_udp_bridge.launch.py
```

Bu depoyu doğrudan çalışma alanı olarak kullanıyorsanız `cd ~/ros2_ws` yerine
`cd /home/mahmut/Desktop/amr_jetson/Jetson_AMR` kullanın ve aşağıdaki install
source yollarını aynı dizine göre değiştirin.

## Topic kontrolü

Her komutu ayrı Jetson terminalinde, ortamı yükledikten sonra çalıştırın:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /plc/mission_command
```

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /plc/raw_rx
```

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /plc/connected --qos-reliability reliable --qos-durability transient_local
```

Son bağlantı durumunu hemen almak için echo'da transient-local seçeneğini
kullanın. Başlangıçta `false`, ilk geçerli cevapla `true` görünür.
Simülatör kapandıktan sonra son geçerli RX üzerinden 2.5 saniye geçince
(timer kontrol aralığı nedeniyle yaklaşık 2.5–2.7 saniye) `false` görünür.
Hatalı boyut veya alan içeren cevap bağlantı zaman aşımını yenilemez ve
mission/raw topic'lerine yayınlanmaz.

## Beklenen çıktı

TF henüz yokken X/Y sıfırdır; uyarı en fazla 5 saniyede bir yazılır.
Geçerli TF elde edildikten sonra geçici TF hatalarında son konum korunur.
TX alım/bırakma başlangıç değerleri `1` ve `2`'dir; mevcut `/plc/tx_pickup`,
`/plc/tx_dropoff`, `/plc/tx_status` topic'leri bunları günceller.

Jetson (X=1 m, Y=-2 m örneği):

```text
TX 7 byte: 01 01 02 64 00 38 ff #1
RX 3 byte: 01 02 02 #1
```

PLC bilgisayarı (kaynak port örnektir):

```text
#1 Robot=172.20.10.2:51717 | RX 7 byte: 01 01 02 64 00 38 ff | status=1, pickup=A1, dropoff=B2, X=1.00 m, Y=-2.00 m
TX 3 byte: 01 02 02
```

Mission/raw topic'lerinde `data: [1, 2, 2]` görünür. `set 1 2 1` sonrasında
`[1, 2, 1]`, `set 1 2 2` sonrasında `[1, 2, 2]` görünmelidir.
Robot yaklaşık saniyede bir tam 7 byte gönderir; simülatör 3 byte cevap verir.

## Ubuntu güvenlik duvarı

PLC bilgisayarında UFW etkinse:

```bash
sudo ufw allow from 172.20.10.2 to any port 1515 proto udp
sudo ufw status
```

Jetson kaynak portu dinamiktir; Jetson'a sadece UDP 1515 açmak cevaplar için
uygun değildir. UFW genellikle giden trafiğin cevaplarına izin verir. Özel
kurallar cevapları engelliyorsa PLC kaynak portuna göre izin verilebilir:

```bash
sudo ufw allow proto udp from 172.20.10.6 port 1515 to 172.20.10.2
```
