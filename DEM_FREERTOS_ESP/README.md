# ESP32 UART - MQTT Ağ Geçidi - Algoritma Akışı
# ESP32 UART to MQTT Gateway - Algorithm Flow

---

## 🇹🇷 Türkçe

Bu ESP-IDF uygulaması, STM32 mikrodenetleyicisinden alınan sensör verilerini bir MQTT sunucusuna iletmek için bir köprü görevi görür.
Sistem başlatıldığında, önce NVS (Kalıcı Depolama) ve iki adet UART portu yapılandırılır.
UART0, bilgisayar üzerinden hata ayıklama (debug) için kullanılırken, UART2, STM32'den veri almak için ayrılır.
Daha sonra Wi-Fi bağlantısı başlatılır ve cihazın geçerli bir IP adresi alması beklenir (Event Group kullanılarak engellemeli bekleme).
IP adresi alındıktan sonra, MQTT istemcisi başlatılır ve HiveMQ genel sunucusuna bağlanır.
Uygulama, veri akışını yönetmek için iki ana FreeRTOS görevi (Task) oluşturur.
Yüksek öncelikli "RX Görevi", sürekli olarak UART2 portunu dinler.
STM32'den gelen veriyi ("P:.. R:.." formatında) okur ve ayrıştırır (parsing).
Veri formatı doğruysa, paylaşılan global değişkenleri (Pitch, Roll, Sıcaklık, Işık) günceller ve durum LED'ini kısa süreliğine yakar.
Normal öncelikli "MQTT Yayınlama Görevi" ise her saniye çalışır.
Bu görev, paylaşılan global değişkenlerdeki en son verileri alır ve bir JSON dizesi oluşturur.
Eğer MQTT bağlantısı aktifse, oluşturulan bu JSON verisini belirlenen konuya (topic) yayınlar.
Wi-Fi veya MQTT bağlantısı kesilirse, ilgili olay işleyicileri (event handlers) durumu algılar ve otomatik olarak yeniden bağlanmaya çalışır veya yayınlamayı duraklatır.

---

## 🇬🇧 English

This ESP-IDF application serves as a bridge to transmit sensor data received from an STM32 microcontroller to an MQTT broker.
Upon system startup, NVS (Non-Volatile Storage) and two UART ports are configured first.
UART0 is used for debugging via PC, while UART2 is dedicated to receiving data from the STM32.
Subsequently, the Wi-Fi connection is initiated, and the system waits for a valid IP address (blocking wait using an Event Group).
Once the IP address is obtained, the MQTT client is initialized and connects to the HiveMQ public broker.
The application creates two main FreeRTOS tasks to manage the data flow.
The high-priority "RX Task" continuously listens to the UART2 port.
It reads and parses the data (in "P:.. R:.." format) coming from the STM32.
If the data format is valid, it updates the shared global variables (Pitch, Roll, Temperature, Light) and briefly blinks the status LED.
The normal-priority "MQTT Publish Task" runs every second.
This task retrieves the latest data from the shared global variables and constructs a JSON string.
If the MQTT connection is active, it publishes this JSON data to the specified topic.
If the Wi-Fi or MQTT connection is lost, the respective event handlers detect the status and automatically attempt to reconnect or pause publishing.