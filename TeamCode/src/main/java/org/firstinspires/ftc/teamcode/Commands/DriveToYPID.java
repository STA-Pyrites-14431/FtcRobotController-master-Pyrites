package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class DriveToYPID extends CommandBase {

    private final Drive driveS;
    private double targetY;
    private final PIDController pid;

    public DriveToYPID(Drive driveS, double y) {
        this.driveS = driveS;
        this.targetY = y;
        this.pid = new PIDController(0.02,0.0005,0.005);
        pid.setTolerance(0.5);
        addRequirements(driveS);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetPoint(targetY);
    }
    @Override
    public void execute() {
        double currentY = driveS.getY(DistanceUnit.INCH);
        double power = pid.calculate(currentY);
        power = Range.clip(power, -0.7, 0.7);
        driveS.fieldCentricDrive(power,0,0);
        driveS.updateOdom();
    }
    @Override
    public void end (boolean interrupted) {
        driveS.stop();
    }
    @Override
    public boolean isFinished() {
        return pid.atSetPoint();
    }
}
