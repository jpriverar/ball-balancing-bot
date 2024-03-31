import time
from controller import Controller
from image_processing import ImageProcessor
from utils import *
from inverse_kinematics import z_rot


class BallBalancer:
    def __init__(self):
        params = load_calibration_params()
        self.rotation = params['platform']['rotation']
        self.x_controller = Controller(params['controller'])
        self.y_controller = Controller(params['controller'])
        self.image_processor = ImageProcessor(params['platform'], params['ball'])

    
    def __find_elements(self, frame):
        ball_frame = self.image_processor.preprocess_ball_frame(frame)
        #platform_frame = self.image_processor.preprocess_ball_frame(frame)
        ball_pos = self.image_processor.find_ball(ball_frame)
        #platform_pos = self.image_processor.find_platform(platform_frame)
        platform_pos = self.image_processor.get_fixed_platform_center()
        return platform_pos, ball_pos
    

    def __get_error_vec(self, platform_pos, ball_pos):
        error_vec = (np.array([ball_pos]) - np.array([platform_pos])).T
        error_vec_aligned = np.dot(z_rot(np.radians(self.rotation-60))[:2,:2], error_vec)
        return error_vec_aligned

    
    def main_loop(self):
        sock = listen_for_connection('0.0.0.0', 8080)
        count = 0
        start = time.time()
        while True:
            frame_data, address = sock.recvfrom(65536)

            # Convert the frame data back to a NumPy array
            frame_raw = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_raw, 1)

            platform_pos, ball_pos = self.__find_elements(frame)
            if platform_pos and ball_pos: 
                cv2.circle(frame, platform_pos, 1, (0,0,255), -1)
                cv2.circle(frame, ball_pos, 1, (0,255,0), -1)
                error = self.__get_error_vec(platform_pos, ball_pos)
                x_output = self.x_controller.get_output(error[0][0])
                y_output = self.y_controller.get_output(error[1][0])
                data = np.array([[9.0, x_output, y_output]], dtype=np.float32)
                sock.sendto(data.tobytes(), address)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Compute the FPS
            count += 1
            if count % 20 == 0:
                fps = count/(time.time() - start)
                print(f'fps: {fps}')
                count = 0
                start = time.time()


if __name__ == "__main__":
    balancer = BallBalancer()
    balancer.main_loop()