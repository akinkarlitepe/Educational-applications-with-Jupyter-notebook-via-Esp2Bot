                                                            # Görüntü işleme ile kameradan robotun tespit ve takibi #

import cv2
import numpy as np
import argparse
from collections import deque
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import threading
from datetime import datetime

class RobotTracker:
    def __init__(self, param1=75, param2=40, min_radius=95, max_radius=195, time_window=20):
        self.param1 = param1
        self.param2 = param2
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.time_window = time_window  # 20 saniye
        
        # Zaman tabanlı veri saklama - 20 saniye boyunca tüm hareket
        self.position_data = []  # [(timestamp, x, y, speed), ...]
        self.trail_positions = []  # Görsel iz için
        
        # Grafik için optimize edilmiş veriler
        self.graph_x_data = []
        self.graph_y_data = []
        self.graph_timestamps = []
        self.graph_speeds = []
        
        # Kalman filtresi için değişkenler
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                 [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                               [0, 1, 0, 1],
                                               [0, 0, 1, 0],
                                               [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1
        
        self.is_tracking = False
        self.robot_id = 0
        
        # Performans için değişkenler
        self.last_position = None
        self.lost_frames = 0
        self.max_lost_frames = 10
        
        # Hız ve mesafe hesaplama
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.speed_history = deque(maxlen=20)
        
        # Threading için lock
        self.data_lock = threading.Lock()
        
        # Başlangıç zamanı
        self.start_time = None
        
        # Grafik güncellemesi için flag
        self.graph_updated = False
        
    def clean_old_data(self, current_time):
        """20 saniyeden eski verileri temizle"""
        if self.start_time is None:
            return
            
        cutoff_time = current_time - self.time_window
        
        # Position data temizle
        self.position_data = [data for data in self.position_data 
                             if data[0] >= cutoff_time]
        
        # Trail positions temizle (görsel iz için)
        self.trail_positions = [pos for pos in self.trail_positions 
                               if pos[0] >= cutoff_time]
        
        # Grafik verilerini yeniden oluştur
        if self.position_data:
            self.graph_timestamps = [data[0] for data in self.position_data]
            self.graph_x_data = [data[1] for data in self.position_data]
            self.graph_y_data = [data[2] for data in self.position_data]
            self.graph_speeds = [data[3] for data in self.position_data]
        else:
            self.graph_timestamps = []
            self.graph_x_data = []
            self.graph_y_data = []
            self.graph_speeds = []
    
    def detect_robot(self, frame):
        """Robot tespit etme fonksiyonu"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        
        # HoughCircles ile daire tespit
        circles = cv2.HoughCircles(
            gray_blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=100,
            param1=self.param1,
            param2=self.param2,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            return circles
        return None
    
    def update_tracking(self, position):
        """Kalman filtresi ile pozisyon güncelleme"""
        if not self.is_tracking:
            # İlk tespit - Kalman filtresini başlat
            self.kalman.statePre = np.array([position[0], position[1], 0, 0], np.float32)
            self.kalman.statePost = np.array([position[0], position[1], 0, 0], np.float32)
            self.is_tracking = True
        
        # Ölçüm güncelleme
        measurement = np.array([[position[0]], [position[1]]], np.float32)
        self.kalman.correct(measurement)
        
        # Tahmin
        prediction = self.kalman.predict()
        
        return (int(prediction[0]), int(prediction[1]))
    
    def find_closest_robot(self, circles, last_pos=None):
        """En yakın robotu bulma (eğer birden fazla daire tespit edilirse)"""
        if last_pos is None:
            return circles[0]  # İlk tespit
        
        min_dist = float('inf')
        closest_circle = circles[0]
        
        for circle in circles:
            dist = np.sqrt((circle[0] - last_pos[0])**2 + (circle[1] - last_pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_circle = circle
        
        return closest_circle
    
    def update_position_data(self, position):
        """Pozisyon verilerini güncelle - 20 saniye boyunca sakla"""
        with self.data_lock:
            current_time = time.time()
            
            # Başlangıç zamanını ayarla
            if self.start_time is None:
                self.start_time = current_time
            
            # Hız hesaplama
            speed = 0.0
            if len(self.position_data) > 0:
                last_data = self.position_data[-1]
                dx = position[0] - last_data[1]
                dy = position[1] - last_data[2]
                dt = current_time - last_data[0]
                
                if dt > 0:
                    distance = np.sqrt(dx**2 + dy**2)
                    speed = distance / dt
                    self.total_distance += distance
            
            # Yeni veriyi ekle
            self.position_data.append((current_time, position[0], position[1], speed))
            self.trail_positions.append((current_time, position[0], position[1]))
            
            # Hız geçmişini güncelle
            self.speed_history.append(speed)
            self.current_speed = np.mean(list(self.speed_history))
            
            # Eski verileri temizle (20 saniyeden eski)
            self.clean_old_data(current_time)
            
            self.graph_updated = True
    
    def draw_trail(self, frame):
        """20 saniye boyunca olan hareket izini çiz"""
        if len(self.trail_positions) < 2:
            return
        
        current_time = time.time()
        
        # 20 saniye içindeki tüm pozisyonları çiz
        for i in range(1, len(self.trail_positions)):
            pos_time = self.trail_positions[i][0]
            age = current_time - pos_time
            
            if age > self.time_window:
                continue
                
            # Yaşa göre renk ve kalınlık hesapla
            alpha = max(0.1, 1.0 - (age / self.time_window))
            thickness = max(1, int(5 * alpha))
            
            # Renk gradyanı: Yeni -> Mavi, Eski -> Kırmızı
            blue_intensity = int(255 * alpha)
            red_intensity = int(255 * (1 - alpha))
            color = (blue_intensity, 0, red_intensity)
            
            # Çizgiyi çiz
            prev_pos = (int(self.trail_positions[i-1][1]), int(self.trail_positions[i-1][2]))
            curr_pos = (int(self.trail_positions[i][1]), int(self.trail_positions[i][2]))
            
            cv2.line(frame, prev_pos, curr_pos, color, thickness)
        
        # Hareket yönünü göster (son 5 nokta)
        if len(self.trail_positions) >= 5:
            recent_positions = self.trail_positions[-5:]
            for i in range(1, len(recent_positions)):
                pos1 = (int(recent_positions[i-1][1]), int(recent_positions[i-1][2]))
                pos2 = (int(recent_positions[i][1]), int(recent_positions[i][2]))
                
                # Ok çiz
                cv2.arrowedLine(frame, pos1, pos2, (0, 255, 255), 2, tipLength=0.3)
    
    def draw_robot_info(self, frame, position, radius):
        """Robot bilgilerini çizme"""
        x, y = position
        
        # Robot dairesini çiz
        cv2.circle(frame, (x, y), radius, (0, 255, 0), 3)
        cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
        
        # Robot ID ve koordinatları yazdır
        cv2.putText(frame, f"Robot #{self.robot_id}", 
                   (x - 50, y - radius - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"({x}, {y})", 
                   (x - 30, y - radius - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Zaman bilgisi
        if self.start_time:
            elapsed_time = time.time() - self.start_time
            remaining_time = max(0, self.time_window - elapsed_time)
            cv2.putText(frame, f"Tracking Time: {elapsed_time:.1f}s", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Window: {remaining_time:.1f}s remaining", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Hız ve toplam mesafe göster
        cv2.putText(frame, f"Speed: {self.current_speed:.1f}px/s", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Total Distance: {self.total_distance:.1f}px", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"Path Points: {len(self.position_data)}", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

class LiveGraphPlotter:
    def __init__(self, tracker):
        self.tracker = tracker
        
        # Matplotlib'i interactive mode'a al
        plt.ion()
        
        # Figure ve subplotları oluştur
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 1. X-Y pozisyon grafiği (20 saniye boyunca tüm hareket)
        self.ax1.set_title('Robot Movement Path (Last 20 Seconds)', fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('X Position (pixels)')
        self.ax1.set_ylabel('Y Position (pixels)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal', adjustable='box')
        
        # 2. X koordinatı zaman grafiği
        self.ax2.set_title('X Position vs Time (20s Window)', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('Time (seconds)')
        self.ax2.set_ylabel('X Position (pixels)')
        self.ax2.grid(True, alpha=0.3)
        
        # 3. Y koordinatı zaman grafiği
        self.ax3.set_title('Y Position vs Time (20s Window)', fontsize=14, fontweight='bold')
        self.ax3.set_xlabel('Time (seconds)')
        self.ax3.set_ylabel('Y Position (pixels)')
        self.ax3.grid(True, alpha=0.3)
        
        # 4. Hız grafiği
        self.ax4.set_title('Speed vs Time (20s Window)', fontsize=14, fontweight='bold')
        self.ax4.set_xlabel('Time (seconds)')
        self.ax4.set_ylabel('Speed (pixels/second)')
        self.ax4.grid(True, alpha=0.3)
        
        # Çizgi nesneleri
        self.line_xy, = self.ax1.plot([], [], 'b-', linewidth=2, alpha=0.8, label='Path')
        self.point_current, = self.ax1.plot([], [], 'ro', markersize=12, label='Current Position')
        self.point_start, = self.ax1.plot([], [], 'go', markersize=10, label='Start Point')
        
        # Zaman tabanlı renk gradienti için scatter plot
        self.scatter = self.ax1.scatter([], [], c=[], cmap='viridis', s=20, alpha=0.6, label='Time Gradient')
        
        self.line_x, = self.ax2.plot([], [], 'r-', linewidth=2, label='X Position')
        self.line_y, = self.ax3.plot([], [], 'b-', linewidth=2, label='Y Position')
        self.line_speed, = self.ax4.plot([], [], 'g-', linewidth=2, label='Speed')
        
        # Legendları ekle
        self.ax1.legend(loc='upper right')
        self.ax2.legend()
        self.ax3.legend()
        self.ax4.legend()
        
        # Başlangıç limitleri
        self.ax1.set_xlim(0, 800)
        self.ax1.set_ylim(0, 600)
        
        plt.tight_layout()
        plt.show(block=False)
        
    def update_plot(self):
        """Grafiği güncelle - 20 saniye boyunca tüm hareket"""
        if not self.tracker.graph_updated:
            return
            
        with self.tracker.data_lock:
            if len(self.tracker.graph_x_data) < 1:
                return
            
            x_data = list(self.tracker.graph_x_data)
            y_data = list(self.tracker.graph_y_data)
            timestamps = list(self.tracker.graph_timestamps)
            speeds = list(self.tracker.graph_speeds)
            
            self.tracker.graph_updated = False
            
        if not x_data or not y_data:
            return
        
        # Zaman verilerini relative hale getir
        if self.tracker.start_time is not None:
            time_data = [(t - self.tracker.start_time) for t in timestamps]
        else:
            time_data = list(range(len(timestamps)))
        
        # 1. X-Y path grafiği güncelle
        self.line_xy.set_data(x_data, y_data)
        
        # Zaman tabanlı renk gradiyenti
        if len(time_data) > 0:
            # Eski scatter'ı temizle
            self.scatter.remove()
            
            # Yeni scatter ekle
            colors = time_data  # Zaman değerleri renk için
            self.scatter = self.ax1.scatter(x_data, y_data, c=colors, cmap='viridis', 
                                          s=30, alpha=0.7, label='Time Gradient')
        
        # Current position ve start point
        if len(x_data) > 0:
            self.point_current.set_data([x_data[-1]], [y_data[-1]])
            if len(x_data) > 1:
                self.point_start.set_data([x_data[0]], [y_data[0]])
            
            # Dinamik xlim ve ylim ayarla
            if len(x_data) > 5:
                margin = 50
                x_min, x_max = min(x_data) - margin, max(x_data) + margin
                y_min, y_max = min(y_data) - margin, max(y_data) + margin
                self.ax1.set_xlim(x_min, x_max)
                self.ax1.set_ylim(y_max, y_min)  # Y eksenini ters çevir
        
        # 2. X koordinatı zaman grafiği
        if len(time_data) > 0:
            self.line_x.set_data(time_data, x_data)
            self.ax2.set_xlim(0, self.tracker.time_window)  # 20 saniye sabit
            if len(x_data) > 1:
                x_margin = (max(x_data) - min(x_data)) * 0.1 + 10
                self.ax2.set_ylim(min(x_data) - x_margin, max(x_data) + x_margin)
        
        # 3. Y koordinatı zaman grafiği
        if len(time_data) > 0:
            self.line_y.set_data(time_data, y_data)
            self.ax3.set_xlim(0, self.tracker.time_window)  # 20 saniye sabit
            if len(y_data) > 1:
                y_margin = (max(y_data) - min(y_data)) * 0.1 + 10
                self.ax3.set_ylim(min(y_data) - y_margin, max(y_data) + y_margin)
        
        # 4. Hız grafiği
        if len(speeds) > 0 and len(time_data) > 0:
            self.line_speed.set_data(time_data, speeds)
            self.ax4.set_xlim(0, self.tracker.time_window)  # 20 saniye sabit
            if len(speeds) > 1:
                speed_max = max(speeds) if max(speeds) > 0 else 10
                self.ax4.set_ylim(0, speed_max * 1.2)
        
        # Başlık güncelle - anlık durum bilgisi
        current_time = time.time()
        if self.tracker.start_time:
            elapsed = current_time - self.tracker.start_time
            self.ax1.set_title(f'Robot Movement Path (Last 20s) - Elapsed: {elapsed:.1f}s', 
                             fontsize=14, fontweight='bold')
        
        # Grafikleri güncelle
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
    def run_live_update(self):
        """Canlı güncelleme döngüsü"""
        while True:
            try:
                self.update_plot()
                time.sleep(0.1)  # 10 FPS güncelleme
            except Exception as e:
                print(f"Grafik güncelleme hatası: {e}")
                break

def run_graph_thread(tracker):
    """Grafik thread'i için fonksiyon"""
    plotter = LiveGraphPlotter(tracker)
    plotter.run_live_update()

def main():
    parser = argparse.ArgumentParser(description='20 Saniyelik Robot Hareket Takip Sistemi')
    parser.add_argument('-c', '--camera', type=int, default=1, help='Kamera indeksi (varsayılan: 1)')
    parser.add_argument('--param1', type=int, default=75, help='HoughCircles param1')
    parser.add_argument('--param2', type=int, default=40, help='HoughCircles param2')
    parser.add_argument('--min_radius', type=int, default=95, help='Minimum daire yarıçapı')
    parser.add_argument('--max_radius', type=int, default=195, help='Maksimum daire yarıçapı')
    parser.add_argument('--time_window', type=int, default=20, help='Zaman penceresi (saniye)')
    parser.add_argument('--save_video', action='store_true', help='Videoyu kaydet')
    parser.add_argument('--no_graph', action='store_true', help='Grafiği gösterme')
    
    args = parser.parse_args()
    
    # Kamerayı başlat
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Kamera açılamadı!")
        return
    
    # Video kaydedici (isteğe bağlı)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = None
    if args.save_video:
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        out = cv2.VideoWriter('robot_tracking_20s.avi', fourcc, 20.0, (frame_width, frame_height))
    
    # Robot takipçisini başlat
    tracker = RobotTracker(
        param1=args.param1,
        param2=args.param2,
        min_radius=args.min_radius,
        max_radius=args.max_radius,
        time_window=args.time_window
    )
    
    # Grafik thread'ini başlat (eğer isteniyorsa)
    if not args.no_graph:
        graph_thread = threading.Thread(target=run_graph_thread, args=(tracker,))
        graph_thread.daemon = True
        graph_thread.start()
        time.sleep(2)  # Grafiğin başlatılması için bekle
    
    print(f"Robot takip sistemi başlatıldı ({args.time_window}s zaman penceresi)...")
    print("Çıkmak için 'q' tuşuna basın")
    print("Verileri temizlemek için 'c' tuşuna basın")
    print("Takibi sıfırlamak için 'r' tuşuna basın")
    print("Hareket verilerini kaydetmek için 's' tuşuna basın")
    
    fps_counter = 0
    fps_start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame okunamadı!")
            break
        
        # Robot tespit et
        circles = tracker.detect_robot(frame)
        
        if circles is not None and len(circles) > 0:
            # En yakın robotu seç
            robot_circle = tracker.find_closest_robot(circles, tracker.last_position)
            x, y, r = robot_circle
            
            # Kalman filtresi ile pozisyon güncelle
            predicted_pos = tracker.update_tracking((x, y))
            
            # 20 saniye boyunca pozisyon verilerini sakla
            tracker.update_position_data(predicted_pos)
            tracker.last_position = predicted_pos
            tracker.lost_frames = 0
            
            # Robot bilgilerini çiz
            tracker.draw_robot_info(frame, predicted_pos, r)
            
        else:
            tracker.lost_frames += 1
            if tracker.lost_frames > tracker.max_lost_frames:
                tracker.is_tracking = False
                print("Robot kaybedildi, yeniden aranıyor...")
        
        # 20 saniye boyunca hareket izini çiz
        tracker.draw_trail(frame)
        
        # FPS hesapla ve göster
        fps_counter += 1
        if fps_counter % 30 == 0:
            elapsed_time = time.time() - fps_start_time
            fps = fps_counter / elapsed_time
            cv2.putText(frame, f"FPS: {fps:.1f}", 
                       (frame.shape[1] - 120, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Durum bilgileri
        status = "TRACKING" if tracker.is_tracking else "SEARCHING"
        color = (0, 255, 0) if tracker.is_tracking else (0, 0, 255)
        cv2.putText(frame, f"Status: {status}", 
                   (10, frame.shape[0] - 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Zaman penceresi bilgisi
        cv2.putText(frame, f"Time Window: {args.time_window}s", 
                   (10, frame.shape[0] - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(frame, f"Data Points: {len(tracker.position_data)}", 
                   (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Videoyu kaydet
        if out is not None:
            out.write(frame)
        
        # Görüntüyü göster
        cv2.imshow('Robot Tracker (20s Window)', frame)
        
        # Klavye kontrolleri
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            with tracker.data_lock:
                tracker.position_data.clear()
                tracker.trail_positions.clear()
                tracker.graph_x_data.clear()
                tracker.graph_y_data.clear()
                tracker.graph_timestamps.clear()
                tracker.graph_speeds.clear()
                tracker.speed_history.clear()
                tracker.total_distance = 0.0
                tracker.current_speed = 0.0
                tracker.start_time = None
            print("Tüm hareket verileri temizlendi")
        elif key == ord('r'):
            with tracker.data_lock:
                tracker.position_data.clear()
                tracker.trail_positions.clear()
                tracker.graph_x_data.clear()
                tracker.graph_y_data.clear()
                tracker.graph_timestamps.clear()
                tracker.graph_speeds.clear()
                tracker.speed_history.clear()
                tracker.is_tracking = False
                tracker.lost_frames = 0
                tracker.total_distance = 0.0
                tracker.current_speed = 0.0
                tracker.start_time = None
            print("Takip sistemi sıfırlandı")
        elif key == ord('s'):
            # 20 saniye boyunca hareket verilerini CSV olarak kaydet
            if len(tracker.position_data) > 0:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"robot_path_20s_{timestamp}.csv"
                with open(filename, 'w') as f:
                    f.write("timestamp,relative_time,x,y,speed\n")
                    with tracker.data_lock:
                        for data in tracker.position_data:
                            relative_time = data[0] - tracker.start_time if tracker.start_time else 0
                            f.write(f"{data[0]},{relative_time:.3f},{data[1]},{data[2]},{data[3]:.2f}\n")
                print(f"20 saniye hareket verileri {filename} dosyasına kaydedildi")
                print(f"Toplam {len(tracker.position_data)} veri noktası kaydedildi")
    
    # Temizlik
    cap.release()
    if out is not None:
        out.release()
    cv2.destroyAllWindows()
    print("Program sonlandırıldı")

if __name__ == "__main__":
    main()