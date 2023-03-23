package utilities;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class IMUWrapper {
    private static ADIS16448_IMU m_IMU = new ADIS16448_IMU();
    
    public static void calibrate() {
        m_IMU.calibrate();
    }
    public static double getYAngle() {
        double yAngle = m_IMU.getGyroAngleY();
        System.out.println("y angle: " + yAngle);
        return yAngle;
    }
    public static double getZAngle() {
        return m_IMU.getGyroAngleZ();
    }
}
