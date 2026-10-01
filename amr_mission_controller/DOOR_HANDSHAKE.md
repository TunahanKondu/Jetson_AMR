# K1/K2 PLC kapı beklemesi

Kapı türü graph düğümünün adından değil `arrivalAction` alanından okunur.
Düğüm adı `N1`, `N2` gibi değişmeden kalır. GUI seçenekleri:

| GUI seçimi | JSON değeri | Davranış |
|---|---|---|
| Normal | `None` | Rota devam eder |
| K1 | `DoorK1` | PLC kapı beklemesi başlar |
| K2 | `DoorK2` | PLC kapı beklemesi başlar |

Kapı davranışı görevdeki gerçek yük durumuna göre filtrelenir:

| Yük durumu | Beklenen kapı | Durmadan geçilen kapı |
|---|---|---|
| Yüksüz | K2 | K1 |
| Yüklü | K1 | K2 |

Yük durumu başlangıçta yüksüzdür. `LIFT_UP_COMPLETE` ile yüklü,
`LIFT_DOWN_COMPLETE` ile yeniden yüksüz olur.

QR K1/K2 işaretleri haritadaki fiziksel kapı konumlarını göstermeye devam
eder; graph kapı davranışı QR bağlantısından bağımsızdır.

Rota `DoorK1` veya `DoorK2` işaretli waypoint'e geldiğinde NavigationManager
bir sonraki Nav2 hedefini göndermez. Mission status şu alanları yayınlar:

```json
{
  "automationWaiting": true,
  "waitingDoor": "K1"
}
```

PLC adapter bu durumdan Byte0=`5` üretir. Kapının açılması için adapter,
bekleme başladıktan sonra PLC control byte'ında önce `1`, sonra `2` görmek
zorundadır. Doğru geçişten sonra `/amr/automation_continue` Bool `true`
yayınlanır ve rotanın sonraki waypoint'i gönderilir.

PLC simülatörü otomatik kapı modu açıkken Byte0=`5` paketini görünce önce
control=`1`, `--door-delay` süresi sonunda control=`2` gönderir.

## Lift komutu kabul onayı

Mission Controller `LIFT_UP` veya `LIFT_DOWN` yayınladıktan sonra sırasıyla
`LIFT_UP_ACCEPTED` veya `LIFT_DOWN_ACCEPTED` sonucunu bekler. Bir saniye içinde
kabul gelmezse komut, toplam en fazla üç deneme olacak şekilde yeniden
yayınlanır. Üç denemede de kabul alınmazsa görev `ACTION_ERROR` ile hata
durumuna geçer. Kabul alındıktan sonra normal `LIFT_*_COMPLETE` sonucu beklenir.
