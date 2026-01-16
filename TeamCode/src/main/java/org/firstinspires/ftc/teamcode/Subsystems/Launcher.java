package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher extends SubsystemBase {
    private MotorEx motorLL, motorLR;

    public Launcher(HardwareMap hM) {
        motorLL = new MotorEx(hM,"motorLL");
        motorLR = new MotorEx(hM,"motorLR");

        motorLL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void enableLauncher() {
        motorLL.set(0.4);
        motorLR.set(-0.4);
    }
    public void disableLauncher() {
        motorLL.set(0);
        motorLR.set(0);
    }
}
