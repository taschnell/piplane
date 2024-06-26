import multiprocessing
import demos.crsf_demo as crsf_demo
import legacy.servo_test as servo_test
import imu_pub

def main():
    def run_servo_test():
        servo_test.main()

    # def run_imu():
    #     bo085_pub.main()

    def run_crsf_read():
        crsf_demo.main()

    servo_process = multiprocessing.Process(target=run_servo_test)
    crsf_process = multiprocessing.Process(target=run_crsf_read)
    # imu_process = multiprocessing.Process(target=run_imu)

    servo_process.start()
    crsf_process.start()
    # imu_process.start()

    servo_process.join()
    crsf_process.join()
    # imu_process.join()

if __name__ == "__main__":
    main()
