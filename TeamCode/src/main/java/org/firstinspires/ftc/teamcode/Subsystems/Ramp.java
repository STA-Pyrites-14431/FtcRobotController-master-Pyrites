package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ramp {
    private MotorEx motorR;
    private String status = "";

    public Ramp(HardwareMap hwMap) {
        motorR = new MotorEx(hwMap,"motorR");
        motorR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void forward() {
        motorR.set(0.5);
        status = "Forward";
    }
    public void reverse() {
        motorR.set(-0.5);
        status = "Reverse";
    }
    public void disable() {
        motorR.set(0);
        status = "Disabled";
    }
    public String getStatus() {
        return status;
    }
}
