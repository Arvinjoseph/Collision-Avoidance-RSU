print(">>> THREADED RSU VERSION ACTIVE <<<")

from ultralytics import YOLO
import cv2
import math
import numpy as np
import time
import socket
import threading

# ===============================
# CONFIGURATION
# ===============================

MODEL_PATH = r"C:\Users\VICTUS\Downloads\best.pt"

CAMERA_INDEX = 0
CONF_THRESH = 0.20

LICENSE_MAP = {
    "truck": 1,
    "car": 2
}

MAX_DIST = 80
TIMEOUT = 4.0

# ===============================
# ESP32 TCP SERVER CONFIG
# ===============================

RSU_HOST = "0.0.0.0"
RSU_PORT = 5050

# ===============================
# FINAL LANE POLYGONS (LOCKED)
# ===============================

UP_LANE_POLYGON = [
    (200, 120),
    (300, 120),
    (278, 310),
    (127, 320)
]

DOWN_LANE_POLYGON = [
    (425, 188),
    (510, 190),
    (600, 320),
    (460, 320)
]

STOP_Y = 320

# ===============================
# GLOBAL TABLES
# ===============================

vehicles = {}
vehicle_id_counter = 0

# license_no -> {mac, type, last_seen, socket}
esp32_table = {}

# ===============================
# ESP32 TCP LISTENER THREAD
# ===============================

def esp32_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((RSU_HOST, RSU_PORT))
    server.listen(5)

    print(f"✅ RSU TCP SERVER LISTENING ON PORT {RSU_PORT}")

    while True:
        client, addr = server.accept()
        print(f"[ESP32 CONNECTED] {addr}")

        def handle_client(sock):
            while True:
                try:
                    data = sock.recv(1024).decode().strip()
                    if not data:
                        break

                    mac, license_no, vtype = data.split(",")
                    license_no = int(license_no)

                    esp32_table[license_no] = {
                        "mac": mac,
                        "type": vtype.lower(),
                        "last_seen": time.time(),
                        "socket": sock
                    }

                    print(f"[ESP32 DATA] LICENSE={license_no} | TYPE={vtype} | MAC={mac}")

                except Exception:
                    break

            sock.close()
            print("[ESP32 DISCONNECTED]")

        threading.Thread(target=handle_client, args=(client,), daemon=True).start()

threading.Thread(target=esp32_tcp_server, daemon=True).start()

# ===============================
# CAMERA INITIALIZATION
# ===============================

model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("❌ Camera not opened")
    exit()
else:
    print("✅ Camera opened successfully")

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# ===============================
# HELPER FUNCTIONS
# ===============================

def point_in_polygon(cx, cy, polygon):
    poly = np.array(polygon, dtype=np.int32).reshape((-1, 1, 2))
    return cv2.pointPolygonTest(poly, (cx, cy), False) >= 0


def match_vehicle(cx, cy, lane, license_no):
    global vehicle_id_counter

    for vid, v in vehicles.items():
        if v["lane"] == lane and v["license_no"] == license_no:
            return vid

    for vid, v in vehicles.items():
        px, py = v["centroid"]
        if math.hypot(cx - px, cy - py) < MAX_DIST:
            return vid

    vehicle_id_counter += 1
    return vehicle_id_counter


def print_joined_table(vehicles, esp32_table, lead_up, lead_down):
    print("\n================ JOINED VEHICLE TABLE ================")
    print("LICENSE | TYPE   | MAC               | LANE | LEAD")
    print("------------------------------------------------------")

    for vid, v in vehicles.items():
        license_no = v["license_no"]
        lane = v["lane"]

        if license_no not in esp32_table:
            continue

        mac = esp32_table[license_no]["mac"]
        vtype = esp32_table[license_no]["type"]

        lead = "YES" if vid == lead_up or vid == lead_down else "NO"

        print(
            f"{license_no:^7} | "
            f"{vtype:<6} | "
            f"{mac:<17} | "
            f"{lane:^4} | "
            f"{lead:^4}"
        )

    print("======================================================")


# ===============================
# 🚨 SEND ALERTS (FINAL LOGIC)
# ===============================

def send_alerts(vehicles, esp32_table, lead_up, lead_down):

    # UP lead → ALL DOWN lane vehicles
    if lead_up is not None:
        lead = vehicles[lead_up]
        msg = f"ALERT,UP,{lead['label'].upper()},{lead['license_no']}\n"

        for v in vehicles.values():
            if v["lane"] == "DOWN":
                ln = v["license_no"]
                if ln in esp32_table:
                    try:
                        esp32_table[ln]["socket"].sendall(msg.encode())
                        print(f"[RSU] Sent to LICENSE {ln}: {msg.strip()}")
                    except Exception:
                        pass

    # DOWN lead → ALL UP lane vehicles
    if lead_down is not None:
        lead = vehicles[lead_down]
        msg = f"ALERT,DOWN,{lead['label'].upper()},{lead['license_no']}\n"

        for v in vehicles.values():
            if v["lane"] == "UP":
                ln = v["license_no"]
                if ln in esp32_table:
                    try:
                        esp32_table[ln]["socket"].sendall(msg.encode())
                        print(f"[RSU] Sent to LICENSE {ln}: {msg.strip()}")
                    except Exception:
                        pass

# ===============================
# MAIN LOOP
# ===============================

while True:
    ret, frame = cap.read()
    if not ret:
        break

    current_time = time.time()
    results = model(frame, conf=CONF_THRESH, verbose=False)

    for r in results:
        for box in r.boxes:
            cls = int(box.cls[0])
            label = model.names[cls].lower()

            if label not in LICENSE_MAP:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            if point_in_polygon(cx, cy, UP_LANE_POLYGON):
                lane = "UP"
            elif point_in_polygon(cx, cy, DOWN_LANE_POLYGON):
                lane = "DOWN"
            else:
                continue

            license_no = LICENSE_MAP[label]
            vid = match_vehicle(cx, cy, lane, license_no)

            vehicles[vid] = {
                "license_no": license_no,
                "label": label,
                "centroid": (cx, cy),
                "lane": lane,
                "last_seen": current_time
            }

    vehicles = {
        vid: v for vid, v in vehicles.items()
        if current_time - v["last_seen"] <= TIMEOUT
    }

    up = [(vid, v["centroid"][1]) for vid, v in vehicles.items() if v["lane"] == "UP"]
    down = [(vid, v["centroid"][1]) for vid, v in vehicles.items() if v["lane"] == "DOWN"]

    lead_up = max(up, key=lambda x: x[1])[0] if up else None
    lead_down = max(down, key=lambda x: x[1])[0] if down else None

    print_joined_table(vehicles, esp32_table, lead_up, lead_down)

    send_alerts(vehicles, esp32_table, lead_up, lead_down)

    for vid, v in vehicles.items():
        cx, cy = v["centroid"]
        color = (0, 255, 0)
        if vid == lead_up or vid == lead_down:
            color = (0, 0, 255)

        cv2.circle(frame, (cx, cy), 6, color, -1)
        cv2.putText(
            frame,
            f"{v['label'].upper()} | {v['lane']}",
            (cx - 40, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2
        )

    cv2.polylines(frame, [np.array(UP_LANE_POLYGON)], True, (255, 255, 0), 2)
    cv2.polylines(frame, [np.array(DOWN_LANE_POLYGON)], True, (255, 255, 0), 2)
    cv2.line(frame, (0, STOP_Y), (frame.shape[1], STOP_Y), (0, 255, 255), 2)

    cv2.imshow("Collision Avoidance – RSU (FULL INTEGRATION)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
