import sys, time
import numpy as np
from collections import deque
from rplidar import RPLidar, RPLidarException
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from sklearn.cluster import DBSCAN

# —————— PARAMETRI ——————
PORT_NAME = 'COM5'
BAUDRATE = 115200
TIMEOUT = 3  # Aumentato per dare più tempo di risposta
DMAX = 4000

X_MIN, X_MAX = 500, 1500
Y_MIN, Y_MAX = -500, 500

QUALITY_MIN = 3
CLUSTER_EPS = 200
CLUSTER_MIN_POINTS = 6
FRAME_HISTORY = 3
# ———————————————————————

class MotionTracker:
    def __init__(self, max_history=FRAME_HISTORY, cluster_eps=CLUSTER_EPS,
                 min_cluster_points=CLUSTER_MIN_POINTS, quality_min=QUALITY_MIN):
        self.frame_buffer = deque(maxlen=max_history)
        self.cluster_eps = cluster_eps
        self.min_cluster_points = min_cluster_points
        self.quality_min = quality_min

    def preprocess_scan(self, scan):
        try:
            # Filtro i dati in base alla qualità e distanza
            filtered = [(angle, dist) for quality, angle, dist in scan
                        if dist > 0 and quality >= self.quality_min]
            if not filtered:
                return None
            angles, dists = zip(*filtered)
            x, y = self.polar_to_cartesian(np.array(angles), np.array(dists))
            return np.column_stack((x, y))
        except Exception as e:
            print(f"Errore durante il preprocessamento della scansione: {e}")
            return None

    @staticmethod
    def polar_to_cartesian(angles, dists):
        th = np.radians(angles)
        return dists * np.cos(th), dists * np.sin(th)

    def update_frame(self, points):
        self.frame_buffer.append(points)
        if len(self.frame_buffer) < self.frame_buffer.maxlen:
            return None
        stacked = np.stack(self.frame_buffer)
        return np.median(stacked, axis=0)

    def detect_clusters(self, points_smoothed):
        clustering = DBSCAN(eps=self.cluster_eps, min_samples=self.min_cluster_points).fit(points_smoothed)
        print(f"Labels DBSCAN: {clustering.labels_[:5]}")  # Debug: visualizza i primi 5 labels
        return clustering.labels_

    def check_danger_zone(self, points, labels, x_min, x_max, y_min, y_max):
        cluster_ids = set(labels)
        for cluster_id in cluster_ids:
            if cluster_id == -1:
                continue
            cluster_points = points[labels == cluster_id]
            if np.any((x_min <= cluster_points[:, 0]) & (cluster_points[:, 0] <= x_max) &
                      (y_min <= cluster_points[:, 1]) & (cluster_points[:, 1] <= y_max)):
                return True
        return False

    def process_scan(self, scan, x_min, x_max, y_min, y_max):
        points = self.preprocess_scan(scan)
        if points is None:
            return None, None, False
        print(f"Punti grezzi: {points[:5]}")  # Stampa i primi 5 punti grezzi per debug

        # Rimuoviamo la smussatura temporaneamente
        smoothed_points = points
        print(f"Punti smussati (senza smussatura): {smoothed_points[:5]}")  # Mostra i primi 5 punti

        labels = self.detect_clusters(smoothed_points)
        alarm = self.check_danger_zone(smoothed_points, labels, x_min, x_max, y_min, y_max)
        return smoothed_points, labels, alarm

class LidarApp:
    def __init__(self):
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(title="RPLidar - Zona Pericolosa")
        self.win.show()
        self.plot = self.win.addPlot()
        self.scatter = pg.ScatterPlotItem(size=5)
        self.plot.addItem(self.scatter)

        rect = QtWidgets.QGraphicsRectItem(X_MIN, Y_MIN, X_MAX - X_MIN, Y_MAX - Y_MIN)
        rect.setPen(pg.mkPen('r', width=2))
        self.plot.addItem(rect)
        self.plot.setXRange(-DMAX, DMAX)
        self.plot.setYRange(-DMAX, DMAX)

        self.tracker = MotionTracker()
        self._connect_lidar()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        print("Sistema avviato. Ctrl+C per uscire.")

    def _connect_lidar(self):
        try:
            if hasattr(self, 'lidar'):
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except:
            pass
        print("Connessione al LiDAR...")
        while True:
            try:
                self.lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=TIMEOUT)
                print("LiDAR connesso.")
                break  # Esci dal ciclo se la connessione ha avuto successo
            except Exception as e:
                print(f"Errore durante la connessione: {e}. Riprovo...")
                time.sleep(1)  # Attendi prima di riprovare

    def update(self):
        try:
            time.sleep(0.1)  # Aggiungi un ritardo di 100 ms tra le scansioni
            scan = next(self.lidar.iter_scans(max_buf_meas=False, min_len=50))
            print(f"Scansione ricevuta: {scan[:5]}")  # Mostra i primi 5 valori della scansione

        except (RPLidarException, StopIteration) as e:
            print(f"Errore durante la scansione: {e}")
            try:
                self.lidar.clear_input()
            except:
                pass
            return

        points, labels, alarm = self.tracker.process_scan(scan, X_MIN, X_MAX, Y_MIN, Y_MAX)

        if points is not None and labels is not None:
            print(f"Punti estratti: {points[:5]}")  # Mostra i primi 5 punti estratti
            scatter_data = []
            for i, label in enumerate(labels):
                if label == -1:
                    continue  # Ignoriamo i punti rumore (-1 sono i punti "rumore" da DBSCAN)
                scatter_data.append({'pos': (points[i][0], points[i][1])})

            # Usa l'approccio predefinito per colorare i punti (gestito automaticamente da PyQtGraph)
            self.scatter.setData(scatter_data)

        if alarm:
            print("!!! ALLARME: oggetto in movimento nella zona pericolosa !!!")

    def run(self):
        sys.exit(self.app.exec_())

if __name__ == '__main__':
    app = LidarApp()
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente.")
    finally:
        try:
            app.lidar.stop()
            app.lidar.stop_motor()
            app.lidar.disconnect()
        except:
            pass
        print("LiDAR disconnesso.")
