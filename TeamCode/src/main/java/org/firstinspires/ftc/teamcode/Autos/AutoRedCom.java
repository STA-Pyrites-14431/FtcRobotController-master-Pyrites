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
import org.firstinspires.ftc.teamcode.Commands.DriveToX;
import org.firstinspires.ftc.teamcode.Commands.DriveToXPID;
import org.firstinspires.ftc.teamcode.Commands.DriveToY;
import org.firstinspires.ftc.teamcode.Commands.DriveToYPID;
import org.firstinspires.ftc.teamcode.Commands.TurnToAnglePD;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Subsystems.Ramp;

@Autonomous(name = "AutoRedCom")
public class AutoRedCom extends CommandOpMode {

    Drive driveS;
    Intake intakeS;
    Launcher launcherS;
    Ramp rampS;
    Command t0, t90, x24, y24, x0, y0;
    Command tS;
    Command lE, lD, rE, rD, iE, iD;
    Command x24PID, y24PID, t90PD;
    SequentialCommandGroup shoot, test;
    Pose2D p1, p2, p3, p4, p5, p6; //Little p for points
    SequentialCommandGroup P1, P2, P3, P4, P5, P6; //Big P for paths
    DistanceUnit I = DistanceUnit.INCH;
    AngleUnit D = AngleUnit.DEGREES;
    WaitCommand w;

    @Override
    public void initialize() {
        driveS = new Drive(hardwareMap, telemetry);
        intakeS = new Intake(hardwareMap);
        launcherS = new Launcher(hardwareMap);
        rampS = new Ramp(hardwareMap);

        //basic wait commands
        w = new WaitCommand(200);

        //turn commands
        t0 = new TurnToAnglePD(driveS,0,telemetry);
        t90 = new TurnToAnglePD(driveS,90,telemetry);
        tS = new TurnToAnglePD(driveS,228,telemetry);
        //drive commands
        x0 = new DriveToX(driveS,0);
        x24 = new DriveToX(driveS,24);
        //strafe commands
        y0 = new DriveToY(driveS,0);
        y24 = new DriveToY(driveS,24);

        //non-driving commands
        lE = new InstantCommand(launcherS::enable);
        lD = new InstantCommand(launcherS::disable);
        rE = new InstantCommand(rampS::forward);
        rD = new InstantCommand(rampS::disable);
        iE = new InstantCommand(intakeS::forward);
        iD = new InstantCommand(intakeS::disable);

        //making points
        p1 = new Pose2D(I,24,24,D,223);
        p2 = new Pose2D(I,12,24,D,90);

        Pose2D start = new Pose2D(DistanceUnit.INCH,-60,-24,AngleUnit.DEGREES,0);
//        driveS.setStart(start);

        //putting commands together
        P1 = DriveToPose2D(p1);
        P2 = DriveToPose2D(p2);
        shoot = new SequentialCommandGroup(lE,new WaitCommand(1250),rE,iE,new WaitCommand(2000),lD,rD,iD);
        test = new SequentialCommandGroup(t90,w,t0,w,x24,w,y24,w,x0,w,y0);

        schedule(P1,new WaitCommand(200),shoot,new WaitCommand(200),P2);
    }
    public SequentialCommandGroup DriveToPose2D(Pose2D p) {
        double x = p.getX(I);
        double y = p.getY(I);
        double h = p.getHeading(D);
        Command X = new DriveToXPID(driveS,x,telemetry);
        Command Y = new DriveToYPID(driveS,y,telemetry);
        Command H = new TurnToAnglePD(driveS,h,telemetry);
        return new SequentialCommandGroup(X,new WaitCommand(200),Y,new WaitCommand(200),H);
    }
}
