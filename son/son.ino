#include <ICM_20948.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const char* ssid = "Cankat";
const char* password = "12345678";

ICM_20948_I2C myICM;
Adafruit_ICM20948 icm;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

float yaw = 0, pitch = 0, roll = 0;
float alpha = 0.98; // Complementary filter coefficient

// Motor pinleri
const int PWMA = 27; // Motor A PWM kontrolü
const int AIN1 = 12; // Motor A yön kontrolü 1
const int AIN2 = 14; // Motor A yön kontrolü 2
const int PWMB = 13; // Motor B PWM kontrolü
const int BIN1 = 26; // Motor B yön kontrolü 1
const int BIN2 = 25; // Motor B yön kontrolü 2
const int STBY = 33; // Standby pini

// LED pinleri
const int LED1_PIN = 17;
const int LED2_PIN = 16;
const int LED3_PIN = 4;

String imuDataCollection[100];
int collectionIndex = 0;

// Prototipler
String createJSON();
void sendIMUDataViaWebSocket();
void controlLED(uint8_t ledPin, bool state);
void saveToCollection(String data);
void controlMotorA(int speed, bool direction);
void controlMotorB(int speed, bool direction);
void handleRoot();
void calculateBNOAndSend(uint8_t clientID);
void calculateICMAndSend(uint8_t clientID);

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Wi-Fi bağlantısı kuruluyor...");
    }
    Serial.println("Wi-Fi bağlı!");

    Wire.begin();

    // ICM-20948 başlatma
    if (myICM.begin(Wire, 0x69) != ICM_20948_Stat_Ok) {
        Serial.println("ICM-20948 başlatılamadı, bağlantıları kontrol edin!");
        //while (1);
    }
    Serial.println("ICM-20948 başlatıldı!");

    if (!icm.begin_I2C()) {
      Serial.println("Adafruit ICM20948 başlatılamadı!");
      //while (1);
    }
    Serial.println("Adafruit ICM20948 hazır.");

    // BNO-055 başlatma
    if (!bno.begin()) {
        Serial.println("BNO-055 başlatılamadı, bağlantıları kontrol edin!");
        //while (1);
    }
    Serial.println("BNO-055 başlatıldı!");
    
    // BNO-055 kalibrasyonu
    bno.setExtCrystalUse(true);
    delay(1000);

    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("Web sunucusu başlatıldı!");
    Serial.print("ESP32 IP Adresi: ");
    Serial.println(WiFi.localIP());

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket başlatıldı!");

    // LED pinlerini çıkış olarak ayarla
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);

    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);

    // Motor pinlerini çıkış olarak ayarla
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
}

void loop() {
    delay(10); // 100 ms döngü süresi
    server.handleClient();
    webSocket.loop();
    if (myICM.dataReady()) {
        myICM.getAGMT();
        //calculateICMAndSend();
        //calculateBNOAndSend();
    }
}

void webSocketEvent(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) {
        Serial.printf("WebSocket bağlandı: %u\n", client_num);
    } else if (type == WStype_DISCONNECTED) {
        Serial.printf("WebSocket bağlantısı kesildi: %u\n", client_num);
    } else if (type == WStype_TEXT) {
        String message = String((char*)payload);
        Serial.printf("WebSocket mesajı alındı: %s\n", message.c_str());

        // LED Kontrol
        if (message == "LED1_ON") controlLED(LED1_PIN, true);
        else if (message == "LED1_OFF") controlLED(LED1_PIN, false);
        else if (message == "LED2_ON") controlLED(LED2_PIN, true);
        else if (message == "LED2_OFF") controlLED(LED2_PIN, false);
        else if (message == "LED3_ON") controlLED(LED3_PIN, true);
        else if (message == "LED3_OFF") controlLED(LED3_PIN, false);

        // Motor Kontrol
        else if (message == "MOTOR_A_FORWARD") controlMotorA(255, true);
        else if (message == "MOTOR_A_BACKWARD") controlMotorA(255, false);
        else if (message == "MOTOR_A_STOP") controlMotorA(0, true); 
        else if (message == "MOTOR_B_FORWARD") controlMotorB(255, true);
        else if (message == "MOTOR_B_BACKWARD") controlMotorB(255, false);
        else if (message == "MOTOR_B_STOP") controlMotorB(0, true);
        
        // IMU Kontrol
        else if (message == "ICM_READ") calculateICMAndSend();
        else if (message == "BNO_READ") calculateBNOAndSend();

        else if (message == "SAVE_DATA") {
            String jsonData = createJSON();
            saveToCollection(jsonData);
        }
    }
}

void controlLED(uint8_t ledPin, bool state) {
    digitalWrite(ledPin, state ? HIGH : LOW);
    Serial.printf("LED %d %s\n", ledPin, state ? "Açık" : "Kapalı");
}

void controlMotorA(int speed, bool direction) {
    if (speed == 0) { 
        // Motoru durdur
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    } else if (direction) {
        // Motor ileri
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        // Motor geri
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    analogWrite(PWMA, abs(speed));
    Serial.printf("Motor A kontrol: Yön=%s, Hız=%d\n", direction ? "İleri" : "Geri", speed);
}

void controlMotorB(int speed, bool direction) {
    if (speed == 0) { 
        // Motoru durdur
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    } else if (direction) {
        // Motor ileri
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        // Motor geri
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMB, abs(speed));
    Serial.printf("Motor B kontrol: Yön=%s, Hız=%d\n", direction ? "İleri" : "Geri", speed);
}

void calculateICMAndSend() {
    String jsonData = createJSON();
    webSocket.broadcastTXT(jsonData);
}

void calculateBNOAndSend() {
    // BNO-055'den sensör verilerini al
    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    // Sıcaklığı al
    int8_t temperature = bno.getTemp();

    // Kalibrasyon durumunu al
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    // Euler açılarını normalize et (0-1 arası)
    float normYaw = orientationData.orientation.x / 360.0;
    float normPitch = (orientationData.orientation.y + 90) / 180.0;
    float normRoll = (orientationData.orientation.z + 180) / 360.0;

    // JSON formatında BNO verilerini oluştur ve gönder
    String bnoData = "{";
    bnoData += "\"type\":\"BNO\",";
    bnoData += "\"euler\":{\"x\":" + String(orientationData.orientation.x) + ",\"y\":" + String(orientationData.orientation.y) + ",\"z\":" + String(orientationData.orientation.z) + "},";
    bnoData += "\"gyro\":{\"x\":" + String(angVelocityData.gyro.x) + ",\"y\":" + String(angVelocityData.gyro.y) + ",\"z\":" + String(angVelocityData.gyro.z) + "},";
    bnoData += "\"accel\":{\"x\":" + String(accelerometerData.acceleration.x) + ",\"y\":" + String(accelerometerData.acceleration.y) + ",\"z\":" + String(accelerometerData.acceleration.z) + "},";
    bnoData += "\"linearAccel\":{\"x\":" + String(linearAccelData.acceleration.x) + ",\"y\":" + String(linearAccelData.acceleration.y) + ",\"z\":" + String(linearAccelData.acceleration.z) + "},";
    bnoData += "\"mag\":{\"x\":" + String(magnetometerData.magnetic.x) + ",\"y\":" + String(magnetometerData.magnetic.y) + ",\"z\":" + String(magnetometerData.magnetic.z) + "},";
    bnoData += "\"gravity\":{\"x\":" + String(gravityData.acceleration.x) + ",\"y\":" + String(gravityData.acceleration.y) + ",\"z\":" + String(gravityData.acceleration.z) + "},";
    bnoData += "\"temperature\":" + String(temperature) + ",";
    bnoData += "\"calibration\":{\"system\":" + String(system) + ",\"gyro\":" + String(gyro) + ",\"accel\":" + String(accel) + ",\"mag\":" + String(mag) + "},";
    bnoData += "\"normYaw\":" + String(normYaw) + ",";
    bnoData += "\"normPitch\":" + String(normPitch) + ",";
    bnoData += "\"normRoll\":" + String(normRoll);
    bnoData += "}";

    webSocket.broadcastTXT(bnoData);
}

void saveToCollection(String data) {
    if (collectionIndex < 100) {
        imuDataCollection[collectionIndex++] = data;
        Serial.printf("Veri koleksiyona kaydedildi: %s\n", data.c_str());
    } else {
        Serial.println("Koleksiyon dolu! Daha fazla veri kaydedilemiyor.");
    }
}

void handleRoot() {
    String html = R"rawliteral(
        <!DOCTYPE html>
        <html>

        <head>
            <title>Admin Panel</title>
            <script>
                const username = "giris123";
                const password = "bitirme123";

                let socket = new WebSocket("ws://" + location.hostname + ":81/");
                let icmData = {};
                let bnoData = {};

                socket.onmessage = function(event) {
                    let data = JSON.parse(event.data);
                    
                    // Gelen verinin tipine göre işle
                    if (data.type === "ICM") {
                        icmData = data;
                        updateICMDisplay();
                    } else if (data.type === "BNO") {
                        bnoData = data;
                        updateBNODisplay();
                    } else {
                        // Eski format (genel IMU verileri)
                        document.getElementById("accel").textContent = "Accel: X=" + data.accel.x + ", Y=" + data.accel.y + ", Z=" + data.accel.z;
                        document.getElementById("gyro").textContent = "Gyro: X=" + data.gyro.x + ", Y=" + data.gyro.y + ", Z=" + data.gyro.z;
                        document.getElementById("magnetometer").textContent = "Magnetometer: X=" + data.magnetometer.x + ", Y=" + data.magnetometer.y + ", Z=" + data.magnetometer.z;
                        document.getElementById("temperature").textContent = "Temperature: " + data.temperature + " °C";
                    }
                };

                function updateICMDisplay() {
                    // Eski format (genel IMU verileri)
                    document.getElementById("accel").textContent = "Accel: X=" + data.accel.x + ", Y=" + data.accel.y + ", Z=" + data.accel.z;
                    document.getElementById("gyro").textContent = "Gyro: X=" + data.gyro.x + ", Y=" + data.gyro.y + ", Z=" + data.gyro.z;
                    document.getElementById("magnetometer").textContent = "Magnetometer: X=" + data.magnetometer.x + ", Y=" + data.magnetometer.y + ", Z=" + data.magnetometer.z;
                    document.getElementById("temperature").textContent = "Temperature: " + data.temperature + " °C";
                }

                function updateBNODisplay() {
                    if (bnoData.euler) {
                        document.getElementById("bno-euler").textContent = "Euler: X=" + bnoData.euler.x.toFixed(2) + "°, Y=" + bnoData.euler.y.toFixed(2) + "°, Z=" + bnoData.euler.z.toFixed(2) + "°";
                        document.getElementById("bno-gyro").textContent = "Gyro: X=" + bnoData.gyro.x.toFixed(2) + ", Y=" + bnoData.gyro.y.toFixed(2) + ", Z=" + bnoData.gyro.z.toFixed(2);
                        document.getElementById("bno-accel").textContent = "Accel: X=" + bnoData.accel.x.toFixed(2) + ", Y=" + bnoData.accel.y.toFixed(2) + ", Z=" + bnoData.accel.z.toFixed(2);
                        document.getElementById("bno-linear").textContent = "Linear Accel: X=" + bnoData.linearAccel.x.toFixed(2) + ", Y=" + bnoData.linearAccel.y.toFixed(2) + ", Z=" + bnoData.linearAccel.z.toFixed(2);
                        document.getElementById("bno-mag").textContent = "Mag: X=" + bnoData.mag.x.toFixed(2) + ", Y=" + bnoData.mag.y.toFixed(2) + ", Z=" + bnoData.mag.z.toFixed(2);
                        document.getElementById("bno-gravity").textContent = "Gravity: X=" + bnoData.gravity.x.toFixed(2) + ", Y=" + bnoData.gravity.y.toFixed(2) + ", Z=" + bnoData.gravity.z.toFixed(2);
                        document.getElementById("bno-temp").textContent = "Temperature: " + bnoData.temperature + " °C";
                        document.getElementById("bno-calibration").textContent = "Calibration - Sys:" + bnoData.calibration.system + ", Gyro:" + bnoData.calibration.gyro + ", Accel:" + bnoData.calibration.accel + ", Mag:" + bnoData.calibration.mag;
                    }
                }

                function sendCommand(command){
                    socket.send(command);
                }

                function login() {
                    const enteredUsername = document.getElementById('username').value;
                    const enteredPassword = document.getElementById('password').value;

                    if (enteredUsername === username && enteredPassword === password) {
                        document.getElementById('loginPage').style.display = 'none';
                        document.getElementById('mainPage').style.display = 'block';
                        showPage('demo1');
                    } else {
                        alert('Invalid username or password!');
                    }
                }

                function showPage(pageId) {
                    document.querySelectorAll('.content').forEach(page => page.classList.add('hidden'));
                    document.getElementById(pageId).classList.remove('hidden');
                }

                window.onload = function() {
                    document.getElementById('loginPage').classList.add('hidden');
                    document.getElementById('mainPage').classList.remove('hidden');
                };

            </script>
        </head>

        <body>
            <div id="loginPage" class="login-form">
                <h2>Admin Login</h2>
                <input type="text" id="username" placeholder="Username">
                <input type="password" id="password" placeholder="Password">
                <button onclick="login()">Login</button>
            </div>

            <div id="mainPage">
                <div class="navbar">
                    <a href="#"><button onclick="showPage('demo1')">IMU Verileri</button></a>
                    <a href="#"><button onclick="showPage('demo2')">LED</button></a>
                    <a href="#"><button onclick="showPage('demo3')">Motor</button></a>
                </div>

                <div id="demo1" class="content hidden">
                    <h1>IMU Kontrol Paneli</h1>
                    
                    <div class="imu-section">
                        <h2>ICM-20948 Verileri</h2>
                        <div class="data-display">
                            <p id="accel">Accel: Veri bekleniyor...</p>
                            <p id="gyro">Gyro: Veri bekleniyor....</p>
                            <p id="magnetometer">Magnetometer: Veri bekleniyor...</p>
                            <p id="temperature">Temperature: Veri bekleniyor...</p>
                        </div>
                        <button onclick="sendCommand('SAVE_DATA')">Veriyi Kaydet</button>
                    </div>

                    <div class="imu-section">
                        <h2>BNO-055 Verileri</h2>
                        <div class="data-display">
                            <p id="bno-euler">Euler: Veri bekleniyor...</p>
                            <p id="bno-gyro">Gyro: Veri bekleniyor...</p>
                            <p id="bno-accel">Accel: Veri bekleniyor...</p>
                            <p id="bno-linear">Linear Accel: Veri bekleniyor...</p>
                            <p id="bno-mag">Mag: Veri bekleniyor...</p>
                            <p id="bno-gravity">Gravity: Veri bekleniyor...</p>
                            <p id="bno-temp">Temperature: Veri bekleniyor...</p>
                            <p id="bno-calibration">Calibration: Veri bekleniyor...</p>
                        </div>
                        <button onclick="sendCommand('SAVE_DATA')">Veriyi Kaydet</button>
                    </div>
                </div>

                <div id="demo2" class="content hidden">
                    <h2>LED Kontrol</h2>
                    <div class="control-group">
                        <h3>LED 1</h3>
                        <button onclick="sendCommand('LED1_ON')">LED1 Ac</button>
                        <button onclick="sendCommand('LED1_OFF')">LED1 Kapat</button>
                    </div>
                    <div class="control-group">
                        <h3>LED 2</h3>
                        <button onclick="sendCommand('LED2_ON')">LED2 Ac</button>
                        <button onclick="sendCommand('LED2_OFF')">LED2 Kapat</button>
                    </div>
                    <div class="control-group">
                        <h3>LED 3</h3>
                        <button onclick="sendCommand('LED3_ON')">LED3 Ac</button>
                        <button onclick="sendCommand('LED3_OFF')">LED3 Kapat</button>
                    </div>
                </div>

                <div id="demo3" class="content hidden">
                    <h2>Motor Kontrol</h2>
                    <div class="control-group">
                        <h3>Motor A</h3>
                        <button onclick="sendCommand('MOTOR_A_FORWARD')">Motor A ileri</button>
                        <button onclick="sendCommand('MOTOR_A_BACKWARD')">Motor A Geri</button>
                        <button onclick="sendCommand('MOTOR_A_STOP')">Motor A Durdur</button>
                    </div>
                    <div class="control-group">
                        <h3>Motor B</h3>
                        <button onclick="sendCommand('MOTOR_B_FORWARD')">Motor B ileri</button>
                        <button onclick="sendCommand('MOTOR_B_BACKWARD')">Motor B Geri</button>
                        <button onclick="sendCommand('MOTOR_B_STOP')">Motor B Durdur</button>
                    </div>
                </div>
            </div>

            <style>
                body {
                    font-family: Arial, sans-serif;
                    margin: 0;
                    padding: 0;
                    background: #f9f9f9;
                    color: #333;
                }

                h1, h2, h3 {
                    color: #444;
                }

                .hidden {
                    display: none;
                }

                /* Login Form */
                .login-form {
                    width: 100%;
                    height: 100vh;
                    display: flex;
                    justify-content: center;
                    align-items: center;
                    flex-direction: column;
                    background: linear-gradient(135deg, #e3f2fd, #90caf9);
                }

                .login-form input {
                    width: 250px;
                    margin: 10px 0;
                    padding: 10px;
                    border: 1px solid #ccc;
                    border-radius: 5px;
                }

                .login-form button {
                    background-color: #1976d2;
                    color: white;
                    border: none;
                    padding: 10px 20px;
                    border-radius: 5px;
                    cursor: pointer;
                }

                .login-form button:hover {
                    background-color: #1565c0;
                }

                /* Main Page */
                .navbar {
                    background-color: #1976d2;
                    padding: 10px;
                    display: flex;
                    justify-content: center;
                    gap: 10px;
                }

                .navbar button {
                    background-color: #64b5f6;
                    color: white;
                    border: none;
                    padding: 10px 15px;
                    border-radius: 5px;
                    cursor: pointer;
                    font-size: 14px;
                }

                .navbar button:hover {
                    background-color: #42a5f5;
                }

                /* Content */
                .content {
                    padding: 20px;
                    background: white;
                    margin: 20px auto;
                    width: 80%;
                    box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
                    border-radius: 8px;
                }

                .content button {
                    background-color: #29b6f6;
                    color: white;
                    border: none;
                    padding: 10px 15px;
                    margin: 5px;
                    border-radius: 5px;
                    cursor: pointer;
                }

                .content button:hover {
                    background-color: #0288d1;
                }

                /* IMU Sections */
                .imu-section {
                    margin: 20px 0;
                    padding: 15px;
                    border: 2px solid #e0e0e0;
                    border-radius: 8px;
                    background-color: #fafafa;
                }

                .imu-section h2 {
                    margin-top: 0;
                    color: #1976d2;
                }

                .imu-btn {
                    background-color: #4caf50 !important;
                    font-weight: bold;
                    margin-bottom: 10px !important;
                }

                .imu-btn:hover {
                    background-color: #45a049 !important;
                }

                .data-display {
                    background-color: #ffffff;
                    padding: 10px;
                    border-radius: 5px;
                    border: 1px solid #ddd;
                    margin-top: 10px;
                }

                .data-display p {
                    margin: 5px 0;
                    font-family: 'Courier New', monospace;
                    font-size: 14px;
                    color: #333;
                }

                .control-group {
                    margin: 15px 0;
                    padding: 10px;
                    border: 1px solid #ddd;
                    border-radius: 5px;
                    background-color: #fafafa;
                }

                .control-group h3 {
                    margin-top: 0;
                    color: #1976d2;
                }

            </style>
            
        </body>
        </html>
    )rawliteral";
    server.send(200, "text/html", html);
}

String createJSON() {
    String jsonData = "{";
    jsonData += "\"accel\": {\"x\": " + String(myICM.accX()) + ", \"y\": " + String(myICM.accY()) + ", \"z\": " + String(myICM.accZ()) + "},";
    jsonData += "\"gyro\": {\"x\": " + String(myICM.gyrX()) + ", \"y\": " + String(myICM.gyrY()) + ", \"z\": " + String(myICM.gyrZ()) + "},";
    jsonData += "\"magnetometer\": {\"x\": " + String(myICM.magX()) + ", \"y\": " + String(myICM.magY()) + ", \"z\": " + String(myICM.magZ()) + "},";
    jsonData += "\"temperature\": " + String(myICM.temp()) + "}";
    return jsonData;
}

