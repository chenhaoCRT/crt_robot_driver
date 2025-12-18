import cv2
import numpy as np
import math

cap = cv2.VideoCapture('/dev/video0')

angle = 0  # 动态旋转元素

while True:
    ret, frame = cap.read()
    if not ret:
        break

    overlay = frame.copy()
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # --- HUD 基础颜色 ---
    color = (0, 255, 0)

    # ============================================================
    #  1. 半透明 HUD 外环
    # ============================================================
    radius = 140
    cv2.circle(overlay, (cx, cy), radius, color, 2)

    # 刻度（每15°一个）
    for a in range(0, 360, 15):
        x1 = int(cx + radius * math.cos(math.radians(a)))
        y1 = int(cy + radius * math.sin(math.radians(a)))

        x2 = int(cx + (radius - 15) * math.cos(math.radians(a)))
        y2 = int(cy + (radius - 15) * math.sin(math.radians(a)))

        cv2.line(overlay, (x1, y1), (x2, y2), color, 1)

    # ============================================================
    #  2. 内圈十字瞄准 + 中心点
    # ============================================================
    cv2.line(overlay, (cx - 40, cy), (cx + 40, cy), color, 1)
    cv2.line(overlay, (cx, cy - 40), (cx, cy + 40), color, 1)

    cv2.circle(overlay, (cx, cy), 6, color, 2)
    cv2.circle(overlay, (cx, cy), 3, color, -1)

    # ============================================================
    #  3. 动态旋转的能量条（科幻HUD灵魂）
    # ============================================================
    angle += 4
    small_r = 80

    rx = int(cx + small_r * math.cos(math.radians(angle)))
    ry = int(cy + small_r * math.sin(math.radians(angle)))

    cv2.circle(overlay, (rx, ry), 10, color, 2)
    cv2.circle(overlay, (rx, ry), 4, color, -1)

    # ============================================================
    # 4. 飞机式俯仰刻度（水平线 HUD）
    # ============================================================
    for offset in range(-60, 61, 20):
        y = cy + offset
        line_len = 60 - abs(offset) // 2
        cv2.line(overlay, (cx - line_len, y), (cx + line_len, y), color, 1)

    # ============================================================
    # 5. 外侧科技线
    # ============================================================
    cv2.line(overlay, (cx - radius - 40, cy), (cx - radius, cy), color, 2)
    cv2.line(overlay, (cx + radius, cy), (cx + radius + 40, cy), color, 2)

    cv2.line(overlay, (cx, cy - radius - 40), (cx, cy - radius), color, 2)
    cv2.line(overlay, (cx, cy + radius), (cx, cy + radius + 40), color, 2)

    # ============================================================
    # 6. 半透明融合
    # ============================================================
    alpha = 0.5
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    # Frame 显示
    cv2.imshow("HUD", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
