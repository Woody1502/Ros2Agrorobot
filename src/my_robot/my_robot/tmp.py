import cv2
import numpy as np

# Создаем черное изображение
img = np.zeros((300, 300), dtype=np.uint8)

# Рисуем белый прямоугольник (контур)
cv2.rectangle(img, (50, 50), (250, 250), 255, -1)

# Находим контуры
contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Вычисляем площадь первого контура
area = cv2.contourArea(contours[0])
print(f"Площадь контура: {area} пикселей")  # Выведет 40000 (200x200)

# Вычисляем момент контура для нахождения центра
M = cv2.moments(contours[0])
if M["m00"] != 0:
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
else:
    cX, cY = 0, 0

print(f"Центр контура: ({cX}, {cY})")

# Создаем цветное изображение для визуализации
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# Рисуем контур зеленым цветом
cv2.drawContours(img_color, contours, -1, (0, 255, 0), 2)

# Рисуем центр контура красным кружком
cv2.circle(img_color, (cX, cY), 5, (0, 0, 255), -1)

# Выводим изображение
cv2.imshow("Contour with Center", img_color)
cv2.waitKey(0)
cv2.destroyAllWindows()