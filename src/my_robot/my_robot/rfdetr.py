#!src/my_robot/venv/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rfdetr import RFDETRBase
import supervision as sv
import cv2
import numpy as np
from sklearn.linear_model import LinearRegression
from std_msgs.msg import Float32 


class DetrDetector(Node):
    def __init__(self):
        super().__init__('rfdetr')
        
        # Загрузка модели YOLO (YOLOv8n)
        self.model = RFDETRBase(pretrain_weights='checkpoint_best_total.pth')
        self.text_scale = sv.calculate_optimal_text_scale(resolution_wh=(800,800))
        self.thickness = sv.calculate_optimal_line_thickness(resolution_wh=(800,800))
        self.bbox_annotator = sv.BoxAnnotator(thickness=self.thickness)
        self.label_annotator = sv.LabelAnnotator(
            text_color=sv.Color.BLACK,
            text_scale=self.text_scale,
            text_thickness=self.thickness,
            smart_position=True)
        self.angle_pub = self.create_publisher(Float32, '/steering_angle', 10)

        # Подписка на камеру
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/pure_image',
            self.image_callback,
            10)
        
        # Публикация результатов
        self.detection_pub = self.create_publisher(Image, '/camera/depth/detections', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Конвертация ROS Image -> OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Детекция YOLO
        results = self.model.predict(cv_image, threshold=0.5)
        centers = []
        
        for det in results.xyxy.tolist():
            # Центр объекта (нормализованные координаты)
            center_x = (((det[0] + det[2]) / 2)/800)-0.5
            center_y = 1-(((det[1] + det[3]) / 2)/800)
            centers.append((center_x, center_y))
        
        angle_msg = Float32()
        
        # Линейная регрессия
        if len(centers) >= 2:  # Нужно хотя бы 2 точки для регрессии
            centers_array = np.array(centers)
            x = centers_array[:, 0].reshape(-1, 1)  # X координаты
            y = centers_array[:, 1]                 # Y координаты
            
            model = LinearRegression()
            model.fit(x, y)
            
            # Коэффициенты линии y = kx + b
            k = model.coef_[0]
            b = model.intercept_
            
            # Преобразование обратно в пиксельные координаты для отрисовки
            img_height, img_width = cv_image.shape[:2]
            
            # Точки для отрисовки линии (крайние точки изображения)
            x1_px = 0
            y1_px = int((1 - (k*(-0.5) + b)) * img_height)
            x2_px = img_width
            y2_px = int((1 - (k*0.5 + b)) * img_height)
            
            # Рисуем линию
            cv2.line(cv_image, (x1_px, y1_px), (x2_px, y2_px), (0, 0, 255), 2)
            
            # Выводим уравнение линии
            equation = f"y = {k:.2f}x + {b:.2f}"
            cv2.putText(cv_image, equation, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 255, 0), 2)
            
            # Вычисляем угол поворота (в градусах)
            angle = np.arctan(k) * 180 / np.pi
            angle_text = f"Angle: {angle:.1f}°"
            cv2.putText(cv_image, angle_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
            angle_rad = np.arctan(k)  # Получаем угол в радианах
            cos_angle = np.cos(angle_rad)  # Вычисляем косинус угла
            if k <0:  # Вычисляем косинус угла
                angle_msg.data = float(-cos_angle)
            else:
                angle_msg.data = float(cos_angle)
            self.get_logger().info(f'{centers}, {angle_msg}')
        elif len(centers) == 1:  # Нужно хотя бы 2 точки для регрессии
            centers.append((0,0))
            centers_array = np.array(centers)
            x = centers_array[:, 0].reshape(-1, 1)  # X координаты
            y = centers_array[:, 1]                 # Y координаты
            
            model = LinearRegression()
            model.fit(x, y)
            
            # Коэффициенты линии y = kx + b
            k = model.coef_[0]
            b = model.intercept_
            
            # Преобразование обратно в пиксельные координаты для отрисовки
            img_height, img_width = cv_image.shape[:2]
            
            # Точки для отрисовки линии (крайние точки изображения)
            x1_px = 0
            y1_px = int((1 - (k*(-0.5) + b)) * img_height)
            x2_px = img_width
            y2_px = int((1 - (k*0.5 + b)) * img_height)
            
            # Рисуем линию
            cv2.line(cv_image, (x1_px, y1_px), (x2_px, y2_px), (0, 0, 255), 2)
            
            # Выводим уравнение линии
            equation = f"y = {k:.2f}x + {b:.2f}"
            cv2.putText(cv_image, equation, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 255, 0), 2)
            
            # Вычисляем угол поворота (в градусах)
            angle = np.arctan(k) * 180 / np.pi
            angle_text = f"Angle: {angle:.1f}°"
            cv2.putText(cv_image, angle_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
            angle_rad = np.arctan(k)  # Получаем угол в радианах
            cos_angle = np.cos(angle_rad)
            if k <0:  # Вычисляем косинус угла
                angle_msg.data = float(-cos_angle)
            else:
                angle_msg.data = float(cos_angle)
            self.get_logger().info(f'{centers}, {angle_msg}')
        else:

            angle_msg.data = float(0)
            self.get_logger().info(f'{centers}, {angle_msg}')

        self.angle_pub.publish(angle_msg)
        # Аннотация детекций
        self.detections_labels = [
            f"куст {confidence:.2f}"
            for class_id, confidence
            in zip(results.class_id, results.confidence)
        ]
        detections_image = self.bbox_annotator.annotate(cv_image, results)
        detections_image = self.label_annotator.annotate(detections_image, results, self.detections_labels)
        
        # Публикация обработанного изображения
        detection_msg = self.bridge.cv2_to_imgmsg(detections_image, "bgr8")
        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    rfdetr = DetrDetector()
    rclpy.spin(rfdetr)
    rfdetr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()