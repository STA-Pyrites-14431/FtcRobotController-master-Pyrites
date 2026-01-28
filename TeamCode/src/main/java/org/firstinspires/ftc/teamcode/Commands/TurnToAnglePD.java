package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class TurnToAnglePD extends CommandBase {
    private Drive driveS;
    private double targetA, heading, speed;
    private final PDController pd;
    private Telemetry tel;


    public TurnToAnglePD(Drive driveS, double A, Telemetry tel) {
        this.driveS = driveS;
        this.targetA = A;
        this.pd = new PDController(0.060,0.001);
        this.tel = tel;
        pd.setTolerance(0.3);
        addRequirements(driveS);
    }

    @Override
    public void initialize() {
//        driveS.resetPose();
        pd.reset();
        pd.setSetPoint(targetA);
    }
    @Override
    public void execute() {
        double currentA = driveS.getH(AngleUnit.DEGREES);
        double power = pd.calculate(currentA);
        power = -Range.clip(power, -0.4, 0.4);
        driveS.fieldCentricDrive(0,0,power);
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
        driveS.updateOdom();
    }
    @Override
    public boolean isFinished() {
        return Math.abs(targetA - driveS.getH(AngleUnit.DEGREES)) < 1.0;
    }
}
