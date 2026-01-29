package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.DriveToXPID;
import org.firstinspires.ftc.teamcode.Commands.DriveToYPID;
import org.firstinspires.ftc.teamcode.Commands.TurnToAnglePD;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Ramp;

@Autonomous(name = "AutoBlueCom")
public class AutoBlueCom extends CommandOpMode {
    Drive driveS;
    Intake intakeS;
    Launcher launcherS;
    Ramp rampS;
    Command lE, lD, rE, rD, iE, iD, RB;
    Command x0,x24,x48,y0,y24,y48,t0,t45,t90, tn90;
    SequentialCommandGroup shoot;
    SequentialCommandGroup shootSCG, D1, D2, D3, D4, endSCG; //Big P for paths
    DistanceUnit I = DistanceUnit.INCH;
    AngleUnit D = AngleUnit.DEGREES;
    WaitCommand w;

    @Override
    public void initialize() {
        driveS = new Drive(hardwareMap, telemetry);
        intakeS = new Intake(hardwareMap);
        launcherS = new Launcher(hardwareMap);
        rampS = new Ramp(hardwareMap);

        w = new WaitCommand(200);

        //non-driving commands
        lE = new InstantCommand(launcherS::enable);
        lD = new InstantCommand(launcherS::disable);
        rE = new InstantCommand(rampS::forward);
        rD = new InstantCommand(rampS::disable);
        iE = new InstantCommand(intakeS::forward);
        iD = new InstantCommand(intakeS::disable);

        //making points
        Pose2D start = new Pose2D(DistanceUnit.INCH,-62,-24,AngleUnit.DEGREES,0);
//        driveS.setStart(start);

        //putting commands together
        shootSCG = DriveXYTurnH(96,12,135);
        endSCG = DriveXYTurnH(0,0,0);
        tn90 = new TurnToAnglePD(driveS,-137,telemetry);
        x0 = new DriveToXPID(driveS,0,telemetry);
        x24 = new DriveToXPID(driveS,24,telemetry);
        x48 = new DriveToXPID(driveS,48,telemetry);
        y0 = new DriveToYPID(driveS,0,telemetry);
        y24 = new DriveToYPID(driveS,-24,telemetry);
        y48 = new DriveToYPID(driveS,-48,telemetry);
        t0 = new TurnToAnglePD(driveS,0,telemetry);
        t90 = new TurnToAnglePD(driveS,-90,telemetry);

        Command x12 = DriveX(12);
        Command xn12 = DriveX(-12);

        Command shoot1 = DriveXYTurnH(96,-12,135);
        Command tn135 = TurnH(-135);
        Command pickupPoint1 = DriveXYTurnH(-24,12,90);


        shoot = new SequentialCommandGroup(lE,new WaitCommand(2000),rE,iE,new WaitCommand(3000),lD,rD,iD);
//        SequentialCommandGroup pickup = new SequentialCommandGroup(iE,new WaitCommand(200),x12,new WaitCommand(200),xn12,iD);

        schedule(new SequentialCommandGroup(shoot1,w,shoot,w,tn135,w,pickupPoint1,w));
    }
    public SequentialCommandGroup DriveXYTurnH(double x, double y, double h) {
        Command X = new DriveToXPID(driveS,x,telemetry);
        Command Y = new DriveToYPID(driveS,y,telemetry);
        Command H = new TurnToAnglePD(driveS,-h,telemetry);
        return new SequentialCommandGroup(X,new WaitCommand(200),Y,new WaitCommand(200),H);
    }
    public Command DriveX(double x) {
        return new DriveToXPID(driveS,x,telemetry);
    }
    public Command DriveY(double y) {
        return new DriveToYPID(driveS,y,telemetry);
    }
    public Command TurnH(double h) {
        return new TurnToAnglePD(driveS,h,telemetry);
    }
}
