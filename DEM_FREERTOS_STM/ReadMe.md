# STM32 FreeRTOS Çoklu Görevli Sensör Yönetimi - Algoritma Akışı
# STM32 FreeRTOS Multi-Tasking Sensor Management - Algorithm Flow

---

## 🇹🇷 Türkçe

Bu uygulama, FreeRTOS üzerinde çalışan çoklu görevli bir sensör izleme ve görüntüleme sistemidir.
Sistem başlatıldığında donanım çevre birimleri ayarlanır ve ardından dört ana görev ile bir yazılım zamanlayıcısı oluşturulur.
Yüksek öncelikli ADC görevi, donanım kesmesiyle senkronize bir şekilde çalışır.
DMA transferi tamamlandığında bir semafor aracılığıyla uyandırılan bu görev, analog verileri okur.
Okunan değerlere göre RGB LED'lerin parlaklığını doğrudan günceller ve hesaplanan sıcaklık/ışık verilerini bir kuyruğa (queue) gönderir.
İvmeölçer görevi, I2C veriyolunu bir Mutex (karşılıklı dışlama) ile koruyarak sensörden ham verileri okur.
Okunan verilerden eğim (roll/pitch) açılarını hesaplar ve bu sonuçları da aynı ortak veri kuyruğuna gönderir.
Ekran (OLED) görevi, bu kuyruğu sürekli olarak dinler.
Kuyruktan gelen verileri (ister ADC ister İvmeölçer kaynaklı olsun) alarak yerel değişkenlerini günceller.
Ardından I2C hattını (yine Mutex korumalı olarak) kullanarak OLED ekranı yeni verilerle tazeler.
Ekran görevi her veri işlediğinde, UART görevine bir görev bildirimi (task notification) gönderir.
Bu bildirimi alan UART görevi, uykudan uyanır ve en güncel sensör değerlerini biçimlendirerek seri port üzerinden dış dünyaya iletir.
Tüm bu işlemlere paralel olarak, arka planda çalışan periyodik bir yazılım zamanlayıcısı mevcuttur.
Bu zamanlayıcı, sensör verilerinden bağımsız olarak bir dizi LED üzerinde "Knight Rider" (yürüyen ışık) animasyonunu bit kaydırma mantığıyla yürütür.


---

## 🇬🇧 English

This application is a multi-tasking sensor monitoring and display system running on FreeRTOS.
Upon system initialization, hardware peripherals are configured, followed by the creation of four main tasks and a software timer.
The high-priority ADC task operates synchronously with hardware interrupts.
Woken up by a semaphore when the DMA transfer completes, this task reads the analog data.
It directly updates the brightness of RGB LEDs based on these readings and sends the calculated temperature/light data to a queue.
The accelerometer task reads raw data from the sensor while protecting the I2C bus with a Mutex.
It calculates tilt (roll/pitch) angles from the readings and sends these results to the same shared data queue.
The display (OLED) task continuously listens to this queue.
Upon receiving data (whether from ADC or Accelerometer sources), it updates its local variables.
It then uses the I2C bus (again, Mutex-protected) to refresh the OLED screen with the new data.
Whenever the display task processes data, it sends a task notification to the UART task.
Upon receiving this notification, the UART task wakes up, formats the latest sensor values, and transmits them externally via the serial port.
Parallel to all these operations, a periodic software timer runs in the background.
This timer executes a "Knight Rider" (scanning light) animation on a set of LEDs using bitwise shifting logic, independent of the sensor data.