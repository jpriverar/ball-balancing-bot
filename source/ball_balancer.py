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
        self.sock = listen_for_connection('0.0.0.0', 8080)

    
    def __find_elements(self, frame):
        ball_frame = self.image_processor.preprocess_ball_frame(frame)
        platform_frame = self.image_processor.preprocess_ball_frame(frame)
        ball_pos = self.image_processor.find_ball(ball_frame)
        platform_pos = self.image_processor.find_platform(platform_frame)
        return platform_pos, ball_pos
    

    def __get_error_vec(self, platform_pos, ball_pos):
        error_vec = (np.array([ball_pos]) - np.array([platform_pos])).T
        error_vec_aligned = np.dot(z_rot(-self.rotation)[:2,:2], error_vec)
        return error_vec_aligned

    
    def loop(self):
        count = 0
        start = time.time()
        while True:
            frame_data = self.sock.recvfrom(65536)[0]

            # Convert the frame data back to a NumPy array
            frame_raw = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_raw, 1)

            platform_pos, ball_pos = self.__find_elements(frame)
            if platform_pos is None or ball_pos is None: continue
            error = self.__get_error_vec(platform_pos, ball_pos)
            print(error)

            # Compute the FPS
            count += 1
            if count % 20 == 0:
                fps = count/(time.time() - start)
                print(f'fps: {fps}')
                count = 0
                start = time.time()


if __name__ == "__main__":
    balancer = BallBalancer()
    balancer.loop()