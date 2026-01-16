package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class TurnToAnglePD extends CommandBase {
    private Drive driveS;
    private double targetA, heading, speed;
    private final PDController pd;

    public TurnToAnglePD(Drive driveS, double A) {
        this.driveS = driveS;
        this.targetA = A;
        this.pd = new PDController(0.015,0.003);
        pd.setTolerance(1);
        addRequirements(driveS);
    }

    @Override
    public void initialize() {
        pd.reset();
        pd.setSetPoint(targetA);
    }
    @Override
    public void execute() {
        double currentA = driveS.getH(AngleUnit.DEGREES);
        double power = pd.calculate(currentA);
        power = Range.clip(power, -0.6, 0.6);
        driveS.fieldCentricDrive(0,0,power);
        driveS.updateOdom();
    }
    @Override
    public void end (boolean interrupted) {
        driveS.stop();
    }
    @Override
    public boolean isFinished() {
        return Math.abs(targetA - driveS.getH(AngleUnit.DEGREES)) < 1.0;
    }
}
