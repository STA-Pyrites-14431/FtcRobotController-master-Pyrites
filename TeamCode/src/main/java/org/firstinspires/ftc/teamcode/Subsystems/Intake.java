package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private MotorEx motorI;
    private String status = "";

    public Intake(HardwareMap hwMap) {
        motorI = new MotorEx(hwMap,"motorI");
        motorI.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void forward() {
        motorI.set(1);
        status = "Forward";
    }
    public void reverse() {
        motorI.set(-1);
        status = "Reverse";
    }
    public void disable() {
        motorI.set(0);
        status = "Disabled";
    }
    public String getStatus() {
        return status;
    }
}
