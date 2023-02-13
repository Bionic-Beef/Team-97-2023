package frc.robot.utilities;

import edu.wpi.first.math.controller.PIDController;

public class GyroPIDController {
    public static PIDController my_PID = new PIDController(0.05, 0, 0);

    //returns motor output
    public static double calculate(double yRotation) {
        return my_PID.calculate(yRotation, 0);
    }
}
