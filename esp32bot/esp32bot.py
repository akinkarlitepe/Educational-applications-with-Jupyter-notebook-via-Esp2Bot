import websocket
import threading
import time
import queue


class ESP32bot:
    def __init__(self, ip_address, port=81):
        """
        ESP32 WebSocket bağlantısı.
        :param ip_address: ESP32'nin IP adresi
        :param port: WebSocket bağlantı noktası (varsayılan 81)
        """
        self.ip_address = ip_address
        self.port = port
        self.ws = None
        self.connected = False
        self.message_queue = queue.Queue()
        self.last_message = None
        self._connect()

    def _connect(self):
        """WebSocket bağlantısını başlat."""
        try:
            self.ws = websocket.WebSocketApp(
                f"ws://{self.ip_address}:{self.port}/",
                on_open=self._on_open,
                on_message=self._on_message,
                on_error=self._on_error,
                on_close=self._on_close
            )
            self.thread = threading.Thread(target=self.ws.run_forever)
            self.thread.daemon = True
            self.thread.start()
            print("WebSocket bağlantısı başlatılıyor...")
            timeout = 5  # Bağlantının maksimum bekleme süresi (saniye)
            start_time = time.time()
            while not self.connected:
                if time.time() - start_time > timeout:
                    raise TimeoutError("WebSocket bağlantısı zaman aşımına uğradı.")
                time.sleep(0.1)
        except Exception as e:
            print(f"WebSocket bağlantısı başarısız: {e}")

    def _on_open(self, ws):
        """WebSocket bağlantısı açıldığında çağrılır."""
        self.connected = True
        print("WebSocket bağlantısı başarılı.")

    def _on_message(self, ws, message):
        """ESP32'den gelen mesajları işler."""
        print(f"ESP32 Mesajı: {message}")
        self.last_message = message
        # Mesajı queue'ya ekle
        try:
            self.message_queue.put(message, block=False)
        except queue.Full:
            # Queue doluysa eski mesajları temizle
            try:
                while True:
                    self.message_queue.get_nowait()
            except queue.Empty:
                pass
            self.message_queue.put(message)

    def _on_error(self, ws, error):
        """WebSocket hata yönetimi."""
        print(f"WebSocket Hatası: {error}")

    def _on_close(self, ws, close_status_code, close_msg):
        """WebSocket bağlantısı kapandığında."""
        self.connected = False
        print("WebSocket bağlantısı kapatıldı.")

    def _send_command(self, command):
        """
        ESP32'ye komut gönderir.
        :param command: Gönderilecek komut
        """
        if self.connected and self.ws:
            try:
                self.ws.send(command)
                print(f"Gönderilen Komut: {command}")
            except Exception as e:
                print(f"Komut gönderilemedi: {e}")
        else:
            print("WebSocket bağlantısı yok veya kapalı. Komut gönderilemiyor.")

    def _wait_for_response(self, timeout=2):
        """
        ESP32'den gelen yanıtı bekler.
        :param timeout: Maksimum bekleme süresi (saniye)
        :return: Gelen mesaj veya None
        """
        try:
            return self.message_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def icm_read(self):
        """IMU sensör verilerini oku."""
        self._send_command("ICM_READ")
        return self._wait_for_response()
    
    def bno_read(self):
        """IMU sensör verilerini oku."""
        self._send_command("BNO_READ")
        return self._wait_for_response()

    def ir_read(self):
        """Infrared sensör verilerini oku."""
        self._send_command("READ_INFRARED")
        return self._wait_for_response()

    def motor_a_forward(self):
        """Motor A'yı ileri çalıştır."""
        self._send_command("MOTOR_A_FORWARD")

    def motor_a_backward(self):
        """Motor A'yı geri çalıştır."""
        self._send_command("MOTOR_A_BACKWARD")

    def motor_a_stop(self):
        """Motor A'yı durdur."""
        self._send_command("MOTOR_A_STOP")

    def motor_b_forward(self):
        """Motor B'yi ileri çalıştır."""
        self._send_command("MOTOR_B_FORWARD")

    def motor_b_backward(self):
        """Motor B'yi geri çalıştır."""
        self._send_command("MOTOR_B_BACKWARD")

    def motor_b_stop(self):
        """Motor B'yi durdur."""
        self._send_command("MOTOR_B_STOP")

    def get_last_message(self):
        """Son alınan mesajı döndürür."""
        return self.last_message

    def close(self):
        """WebSocket bağlantısını kapat."""
        if self.ws:
            try:
                self.ws.close()
                print("WebSocket bağlantısı kapatılıyor.")
            except Exception as e:
                print(f"Bağlantı kapatılırken hata: {e}")