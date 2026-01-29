package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class DriveToXPID extends CommandBase {

    private Drive driveS;
    private double targetX;
    private final PIDController pid;
    private Telemetry tel;

    public DriveToXPID(Drive driveS, double x, Telemetry tel) {
        this.driveS = driveS;
        this.targetX = x;
        this.tel = tel;
        this.pid = new PIDController(0.07, 0.0009, 0.0059);
        pid.setTolerance(0.3);
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
        power = Range.clip(power,-0.5,0.5);
        driveS.fieldCentricDrive(0,power,0);
        driveS.updateOdom();
        tel.addData("XPos: ",driveS.getX(DistanceUnit.INCH));
        tel.addData("YPos: ",driveS.getY(DistanceUnit.INCH));
        tel.addData("Heading: ",driveS.getH(AngleUnit.DEGREES));
        tel.update();
    }
    @Override
    public void end (boolean interrupted) {
        driveS.updateOdom();
        driveS.stop();
        driveS.resetPose();
        driveS.updateOdom();
    }
    @Override
    public boolean isFinished() {
        return pid.atSetPoint();
    }
}
