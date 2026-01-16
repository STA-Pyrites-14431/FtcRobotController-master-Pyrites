package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveS extends SubsystemBase {
    private MotorEx motorFL, motorFR, motorBL, motorBR;
    private MecanumDrive mec;

    public DriveS(HardwareMap hM) {
        motorFL = new MotorEx(hM,"motorFL", Motor.GoBILDA.RPM_312); //EH0
        motorFR = new MotorEx(hM,"motorFR",Motor.GoBILDA.RPM_312); //CH0
        motorBL = new MotorEx(hM,"motorBL",Motor.GoBILDA.RPM_312); //EH1
        motorBR = new MotorEx(hM,"motorBR",Motor.GoBILDA.RPM_312); //CH1


        motorBR.setInverted(true);
        motorBL.setInverted(true);
        motorFL.setInverted(true);

        motorFL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
