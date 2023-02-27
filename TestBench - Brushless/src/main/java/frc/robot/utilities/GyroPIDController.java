package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;

public class GyroPIDController {
    public static PIDController y_PID = new PIDController(0.04, 0, 0.1);
    public static PIDController z_PID = new PIDController(0.004, 0, 0.1);

    //returns motor output
    public static double calculateY(double yRotation) {
        return -y_PID.calculate(yRotation, 0);
    }

    public static double calculateZ(double zRotation) {
        return z_PID.calculate(zRotation, 0);
    }
}