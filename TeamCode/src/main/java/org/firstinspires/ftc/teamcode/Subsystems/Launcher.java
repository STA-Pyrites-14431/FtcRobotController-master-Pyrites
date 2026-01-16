package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {
    private MotorEx motorLL, motorLR;
    private String status = "";

    public Launcher(HardwareMap hwMap) {
        motorLL = new MotorEx(hwMap,"motorLL");
        motorLR = new MotorEx(hwMap,"motorLR");
        motorLL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void enable(double P) {
        motorLL.set(P);
        motorLR.set(P);
        status = "Enabled";
    }
    public void disable() {
        motorLL.set(0);
        motorLR.set(0);
        status = "Disabled";
    }
    public String getStatus() {
        return status;
    }
}
