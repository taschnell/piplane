import multiprocessing
import crsf_read
import servo_test
import pwm_motor
import dshot_motor
import bo085_pub

def main():
    def run_servo_test():
        dshot_motor.main()

    def run_imu():
        bo085_pub.main()

    def run_crsf_read():
        crsf_read.main()

    servo_process = multiprocessing.Process(target=run_servo_test)
    crsf_process = multiprocessing.Process(target=run_crsf_read)
    imu_process = multiprocessing.Process(target=run_imu)

    servo_process.start()
    crsf_process.start()
    imu_process.start()

    servo_process.join()
    crsf_process.join()
    imu_process.join()

if __name__ == "__main__":
    main()
