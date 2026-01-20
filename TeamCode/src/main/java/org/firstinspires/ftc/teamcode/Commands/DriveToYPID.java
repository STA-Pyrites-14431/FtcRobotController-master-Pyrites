package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class DriveToYPID extends CommandBase {

    private final Drive driveS;
    private double targetY;
    private final PIDController pid;
    private Telemetry tel;

    public DriveToYPID(Drive driveS, double y, Telemetry tel) {
        this.driveS = driveS;
        this.targetY = y;
        this.pid = new PIDController(0.12,0.0006,0.007);
        this.tel = tel;
        pid.setTolerance(0.2);
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
        power = -Range.clip(power, -0.5, 0.5);
        driveS.fieldCentricDrive(power,0,0);
        driveS.updateOdom();
        tel.addData("XPos: ",driveS.getX(DistanceUnit.INCH));
        tel.addData("YPos: ",driveS.getY(DistanceUnit.INCH));
        tel.addData("Heading: ",driveS.getH(AngleUnit.DEGREES));
        tel.update();
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
