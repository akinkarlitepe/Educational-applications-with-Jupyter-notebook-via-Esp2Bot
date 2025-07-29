#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

const char* ssid = "Sam";
const char* password = "qqzf7668";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Infrared sensör verileri
uint32_t infrared_data = 0;
uint8_t distance_levels[8] = {0};
uint8_t robot_detected[8] = {0};

// LED pinleri
const int LED1_PIN = 17;
const int LED2_PIN = 16;
const int LED3_PIN = 4;

// Motor pinleri
const int PWMA = 27;
const int AIN1 = 12;
const int AIN2 = 14;
const int PWMB = 13;
const int BIN1 = 26;
const int BIN2 = 25;
const int STBY = 33;

String infraredDataCollection[100];
int collectionIndex = 0;

// Prototipler
String createInfraredJSON();
void sendInfraredDataViaWebSocket();
void controlLED(uint8_t ledPin, bool state);
void saveToCollection(String data);
void controlMotorA(int speed, bool direction);
void controlMotorB(int speed, bool direction);
void handleRoot();
void readInfraredSensor();

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Wi-Fi bağlantısı kuruluyor...");
    }
    Serial.println("Wi-Fi bağlı!");

    // I2C başlat
    Wire.begin();  
    Wire.setClock(400000);
    Serial.println("I2C başlatıldı!");

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
    server.handleClient();
    webSocket.loop();
    
    // Infrared sensör verilerini oku
    readInfraredSensor();
    
    // WebSocket üzerinden veri gönder
    sendInfraredDataViaWebSocket();
    
    delay(100); // 100ms döngü süresi
}

void readInfraredSensor() {
    Wire.requestFrom(12, 4);
    infrared_data = 0;
    
    while(Wire.available()) {
        infrared_data = (infrared_data << 8) & 0xFFFFFFF0;
        infrared_data |= Wire.read();
    }
    
    if (infrared_data > 0) {
        uint32_t temp_data = infrared_data;
        
        // Distance seviyelerini ayıkla (ilk 8 x 3 bit)
        for (uint8_t i = 0; i < 8; i++) {
            distance_levels[i] = temp_data & 7; // Son 3 bit
            temp_data = temp_data >> 3;
        }
        
        // Robot detection verilerini ayıkla (sonraki 8 bit)
        for (uint8_t i = 0; i < 8; i++) {
            robot_detected[i] = temp_data & 1; // Son 1 bit
            temp_data = temp_data >> 1;
        }
        
        // Serial çıktı (debug için)
        //Serial.print("distance:");
        for (uint8_t i = 0; i < 8; i++) {
            //Serial.print(" ");s
            //Serial.print(distance_levels[i]);
        }
        //Serial.print(" robot:");
        for (uint8_t i = 0; i < 8; i++) {
            //Serial.print(" ");
            //Serial.print(robot_detected[i]);
        }
        //Serial.println();
    }
}

void sendInfraredDataViaWebSocket() {
    if (infrared_data > 0) {
        String jsonData = createInfraredJSON();
        webSocket.broadcastTXT(jsonData);
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

        // Veri Kaydet
        else if (message == "SAVE_INFRARED_DATA") {
            String jsonData = createInfraredJSON();
            saveToCollection(jsonData);
        }
        
        // Manuel infrared okuma
        else if (message == "READ_INFRARED") {
            readInfraredSensor();
            String jsonData = createInfraredJSON();
            webSocket.sendTXT(client_num, jsonData);
        }
    }
}

void controlLED(uint8_t ledPin, bool state) {
    digitalWrite(ledPin, state ? HIGH : LOW);
    Serial.printf("LED %d %s\n", ledPin, state ? "Açık" : "Kapalı");
}

void controlMotorA(int speed, bool direction) {
    if (speed == 0) { 
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    } else if (direction) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    analogWrite(PWMA, abs(speed));
    Serial.printf("Motor A kontrol: Yön=%s, Hız=%d\n", direction ? "İleri" : "Geri", speed);
}

void controlMotorB(int speed, bool direction) {
    if (speed == 0) { 
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    } else if (direction) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMB, abs(speed));
    Serial.printf("Motor B kontrol: Yön=%s, Hız=%d\n", direction ? "İleri" : "Geri", speed);
}

void saveToCollection(String data) {
    if (collectionIndex < 100) {
        infraredDataCollection[collectionIndex++] = data;
        Serial.printf("Infrared veri koleksiyona kaydedildi: %s\n", data.c_str());
    } else {
        Serial.println("Koleksiyon dolu! Daha fazla veri kaydedilemiyor.");
    }
}

void handleRoot() {
    String html = R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>Infrared Sensor Kontrol Paneli</title>
            <script>
                const username = "giris123";
                const password = "bitirme123";

                let socket = new WebSocket("ws://" + location.hostname + ":81/");

                socket.onmessage = function(event) {
                    let data = JSON.parse(event.data);
                    
                    // Distance levels göster
                    document.getElementById("distances").textContent = 
                        "Distance Levels: " + data.distance_levels.join(", ");
                    
                    // Robot detection göster
                    document.getElementById("robots").textContent = 
                        "Robot Detection: " + data.robot_detected.join(", ");
                    
                    // Raw data göster
                    document.getElementById("rawdata").textContent = 
                        "Raw Data: " + data.raw_data;
                    
                    // Görsel gösterim güncelle
                    updateVisualDisplay(data.distance_levels, data.robot_detected);
                };

                function updateVisualDisplay(distances, robots) {
                    for(let i = 0; i < 8; i++) {
                        let sensorDiv = document.getElementById("sensor_" + i);
                        let level = distances[i];
                        let robotDetected = robots[i];
                        
                        // Renk kodlaması: Mesafe seviyesine göre
                        let color = getColorByDistance(level);
                        sensorDiv.style.backgroundColor = color;
                        
                        // Robot tespit edilmişse kırmızı border
                        if(robotDetected) {
                            sensorDiv.style.border = "3px solid red";
                        } else {
                            sensorDiv.style.border = "1px solid #ccc";
                        }
                        
                        sensorDiv.innerHTML = "S" + i + "<br>D:" + level + "<br>R:" + robotDetected;
                    }
                }

                function getColorByDistance(level) {
                    // Mesafe seviyesine göre renk döndür (0-7 arası)
                    const colors = [
                        "#ff0000", // 0 - Çok yakın (kırmızı)
                        "#ff4500", // 1 - Yakın (turuncu-kırmızı)
                        "#ff8c00", // 2 - Yakın (turuncu)
                        "#ffd700", // 3 - Orta (altın sarısı)
                        "#ffff00", // 4 - Orta (sarı)
                        "#adff2f", // 5 - Uzak (yeşil-sarı)
                        "#90ee90", // 6 - Uzak (açık yeşil)
                        "#98fb98"  // 7 - Çok uzak (açık yeşil)
                    ];
                    return colors[level] || "#ffffff";
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
                        showPage('infrared');
                    } else {
                        alert('Invalid username or password!');
                    }
                }

                function showPage(pageId) {
                    document.querySelectorAll('.content').forEach(page => page.classList.add('hidden'));
                    document.getElementById(pageId).classList.remove('hidden');
                }

                window.onload = function() {
                    // Test için login sayfasını atla
                    document.getElementById('loginPage').classList.add('hidden');
                    document.getElementById('mainPage').classList.remove('hidden');
                    showPage('infrared');
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
                    <button onclick="showPage('infrared')">Infrared Sensor</button>
                    <button onclick="showPage('led')">LED Kontrol</button>
                    <button onclick="showPage('motor')">Motor Kontrol</button>
                </div>

                <div id="infrared" class="content">
                    <h1>Infrared Sensor Paneli</h1>
                    
                    <div class="sensor-display">
                        <h3>Sensor Durumu (Circular Array)</h3>
                        <div class="sensor-circle">
                            <div class="sensor-box" id="sensor_0">S0</div>
                            <div class="sensor-box" id="sensor_1">S1</div>
                            <div class="sensor-box" id="sensor_2">S2</div>
                            <div class="sensor-box" id="sensor_3">S3</div>
                            <div class="sensor-box" id="sensor_4">S4</div>
                            <div class="sensor-box" id="sensor_5">S5</div>
                            <div class="sensor-box" id="sensor_6">S6</div>
                            <div class="sensor-box" id="sensor_7">S7</div>
                            <div class="center-label">ESP32</div>
                        </div>
                    </div>
                    
                    <div class="data-display">
                        <p id="distances">Distance Levels: Yükleniyor...</p>
                        <p id="robots">Robot Detection: Yükleniyor...</p>
                        <p id="rawdata">Raw Data: Yükleniyor...</p>
                    </div>
                    
                    <div class="controls">
                        <h3>Kontroller</h3>
                        <button onclick="sendCommand('READ_INFRARED')">Manuel Okuma</button>
                        <button onclick="sendCommand('SAVE_INFRARED_DATA')">Veriyi Kaydet</button>
                    </div>
                </div>

                <div id="led" class="content hidden">
                    <h2>LED Kontrol</h2>
                    <button onclick="sendCommand('LED1_ON')">LED1 Ac</button>
                    <button onclick="sendCommand('LED1_OFF')">LED1 Kapat</button>
                    <button onclick="sendCommand('LED2_ON')">LED2 Ac</button>
                    <button onclick="sendCommand('LED2_OFF')">LED2 Kapat</button>
                    <button onclick="sendCommand('LED3_ON')">LED3 Ac</button>
                    <button onclick="sendCommand('LED3_OFF')">LED3 Kapat</button>
                </div>

                <div id="motor" class="content hidden">
                    <h2>Motor Kontrol</h2>
                    <h3>Motor A</h3>
                    <button onclick="sendCommand('MOTOR_A_FORWARD')">Motor A Ileri</button>
                    <button onclick="sendCommand('MOTOR_A_BACKWARD')">Motor A Geri</button>
                    <button onclick="sendCommand('MOTOR_A_STOP')">Motor A Durdur</button>
                    <h3>Motor B</h3>
                    <button onclick="sendCommand('MOTOR_B_FORWARD')">Motor B Ileri</button>
                    <button onclick="sendCommand('MOTOR_B_BACKWARD')">Motor B Geri</button>
                    <button onclick="sendCommand('MOTOR_B_STOP')">Motor B Durdur</button>
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

                .hidden { display: none; }

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
                }

                .navbar button:hover {
                    background-color: #42a5f5;
                }

                .content {
                    padding: 20px;
                    background: white;
                    margin: 20px auto;
                    width: 90%;
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

                .sensor-display {
                    text-align: center;
                    margin: 20px 0;
                }

                .sensor-circle {
                    position: relative;
                    width: 400px;
                    height: 400px;
                    margin: 0 auto;
                    border-radius: 50%;
                    border: 2px solid #ccc;
                }

                .sensor-box {
                    position: absolute;
                    width: 60px;
                    height: 60px;
                    border: 1px solid #ccc;
                    border-radius: 8px;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    font-size: 10px;
                    font-weight: bold;
                    text-align: center;
                    background-color: #f0f0f0;
                }

                /* Sensörleri dairesel olarak konumlandır */
                .sensor-box:nth-child(1) { top: 10px; left: 50%; transform: translateX(-50%); }
                .sensor-box:nth-child(2) { top: 25%; right: 10px; }
                .sensor-box:nth-child(3) { top: 50%; right: 10px; transform: translateY(-50%); }
                .sensor-box:nth-child(4) { bottom: 25%; right: 10px; }
                .sensor-box:nth-child(5) { bottom: 10px; left: 50%; transform: translateX(-50%); }
                .sensor-box:nth-child(6) { bottom: 25%; left: 10px; }
                .sensor-box:nth-child(7) { top: 50%; left: 10px; transform: translateY(-50%); }
                .sensor-box:nth-child(8) { top: 25%; left: 10px; }

                .center-label {
                    position: absolute;
                    top: 50%;
                    left: 50%;
                    transform: translate(-50%, -50%);
                    font-weight: bold;
                    font-size: 16px;
                    color: #1976d2;
                }

                .data-display {
                    background: #f5f5f5;
                    padding: 15px;
                    border-radius: 5px;
                    margin: 20px 0;
                }

                .data-display p {
                    margin: 10px 0;
                    font-family: monospace;
                    font-size: 14px;
                }

                .controls {
                    text-align: center;
                    margin-top: 20px;
                }
            </style>
        </body>
        </html>
    )rawliteral";
    server.send(200, "text/html", html);
}

String createInfraredJSON() {
    String jsonData = "{";
    jsonData += "\"raw_data\": " + String(infrared_data) + ",";
    
    // Distance levels array
    jsonData += "\"distance_levels\": [";
    for (uint8_t i = 0; i < 8; i++) {
        jsonData += String(distance_levels[i]);
        if (i < 7) jsonData += ",";
    }
    jsonData += "],";
    
    // Robot detected array
    jsonData += "\"robot_detected\": [";
    for (uint8_t i = 0; i < 8; i++) {
        jsonData += String(robot_detected[i]);
        if (i < 7) jsonData += ",";
    }
    jsonData += "]";
    
    jsonData += "}";
    return jsonData;
}