package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class DriveToXPID extends CommandBase {

    private Drive driveS;
    private double targetX;
    private final PIDController pid;

    public DriveToXPID(Drive driveS, double x) {
        this.driveS = driveS;
        this.targetX = x;
        this.pid = new PIDController(0.02, 0.0005, 0.005);
        pid.setTolerance(0.5);
        addRequirements(driveS);
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetPoint(targetX);
    }
    @Override
    public void execute() {
        double currentX = driveS.getX(DistanceUnit.INCH);
        double power = pid.calculate(currentX);
        power = Range.clip(power,-0.7,0.7);
        driveS.fieldCentricDrive(0,power,0);
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
