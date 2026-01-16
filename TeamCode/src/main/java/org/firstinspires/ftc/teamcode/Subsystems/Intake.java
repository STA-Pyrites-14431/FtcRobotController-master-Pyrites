package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    private MotorEx motorI;

    public Intake(HardwareMap hM) {
        motorI = new MotorEx(hM,"motorI",Motor.GoBILDA.RPM_223);
        motorI.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void enableIntake() {
        motorI.set(-1);
    }
    public void disableIntake() {
        motorI.set(0);
    }
}
