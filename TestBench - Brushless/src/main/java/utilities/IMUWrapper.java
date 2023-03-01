package utilities;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class IMUWrapper {
    private static ADIS16448_IMU m_IMU = new ADIS16448_IMU();
    
    public static void calibrate() {
        m_IMU.calibrate();
    }
    public static double getYAngle() {
        return m_IMU.getGyroAngleY();
    }
    public static double getZAngle() {
        return m_IMU.getGyroAngleZ();
    }
}
