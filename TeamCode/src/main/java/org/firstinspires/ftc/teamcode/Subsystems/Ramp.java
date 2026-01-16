package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ramp extends SubsystemBase {
    private MotorEx motorR;

    public Ramp(HardwareMap hM) {
        motorR = new MotorEx(hM,"motorR", Motor.GoBILDA.RPM_312);
        motorR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void enableRamp() {
        motorR.set(0.5);
    }

    public void disableRamp() {
        motorR.set(0);
    }
}
