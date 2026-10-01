# LINE_START kabul ve tekrar sistemi

Mission Controller `LINE_START` komutunu `/robot_action` üzerinden yayınlar.
Çizgi controller düğümü, kendisi komutu aldıktan ve detector düğümünden ilk
`/color_line_detected` mesajı geldikten sonra `/amr/robot_action_result`
üzerinden `LINE_START_ACCEPTED` yayınlar.

Kabul mesajı bir saniye içinde gelmezse Mission Controller `LINE_START`
komutunu tekrar yayınlar. Görev `Running` durumundan çıktığında, `LINE_STOP`,
`LINE_COMPLETE` veya `ACTION_ERROR` geldiğinde bekleyen tekrar iptal edilir.

Detector ve controller tekrar gelen `LINE_START` mesajında mevcut takip,
PID ve encoder mesafesi durumunu sıfırlamaz. Controller hazır durumdaysa kabul
mesajını yeniden yayınlar.

`LINE_STOP` komutu da kabul edilene kadar tekrar yayınlanır. Detector
`LINE_DETECTOR_STOPPED`, controller ise `LINE_CONTROLLER_STOPPED` sonucunu
gönderir. Mission Controller iki sonucu da almadan durdurmayı tamamlanmış
saymaz. Çizgi tamamlandıktan sonra istenen `LIFT_UP` veya `LIFT_DOWN`, iki stop
onayı gelene kadar bekletilir.

## İzleme

```bash
ros2 topic echo /robot_action
```

```bash
ros2 topic echo /amr/robot_action_result
```

Beklenen sonuç:

```text
LINE_START
LINE_START_ACCEPTED
```

Görev olmadan bütün akışı test etmek için:

```bash
ros2 topic pub --once /amr/mission_command std_msgs/msg/String \
"{data: 'TEST_LINE_START'}"
```

```bash
ros2 topic pub --once /amr/mission_command std_msgs/msg/String \
"{data: 'TEST_LINE_STOP'}"
```
