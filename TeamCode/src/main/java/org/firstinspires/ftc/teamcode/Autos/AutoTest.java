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

@Autonomous(name = "AutoTest")
public class AutoTest extends CommandOpMode {

    Drive driveS;
    Intake intakeS;
    Launcher launcherS;
    Ramp rampS;
    Command lE, lD, rE, rD, iE, iD;
    Command x0,x24,x48,y0,y24,y48,t0,t45,t90;
    SequentialCommandGroup shoot;
    Pose2D shootPose2D, p1, p2, p3, endPose2D; //Little p for points
    SequentialCommandGroup shootSCG, P1, P2, P3, endSCG; //Big P for paths
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
        shootSCG = DriveToPoint(24,-24,-137);
        P1 = DriveToPoint(12,-24,90);
        P2 = DriveToPoint(-12,-24,90);
        P3 = DriveToPoint(-36,-24,90);
        endSCG = DriveToPoint(0,-24,-90);
        x24 = new DriveToXPID(driveS,24,telemetry);
        y0 = new DriveToYPID(driveS,0,telemetry);
        y24 = new DriveToYPID(driveS,-24,telemetry);
        y48 = new DriveToYPID(driveS,-48,telemetry);
        t90 = new TurnToAnglePD(driveS,-137,telemetry);


        shoot = new SequentialCommandGroup(lE,new WaitCommand(1250),rE,iE,new WaitCommand(2000),lD,rD,iD);

//        schedule(new SequentialCommandGroup(shootSCG,w,shoot,w,P1,shootSCG,w,shoot,w,P2,w,shootSCG,w,shoot,w,P3,w,shootSCG,w,shoot,w,endSCG));
        schedule(new SequentialCommandGroup(y24,new WaitCommand(2000),y0));

    }
    public SequentialCommandGroup DriveToPoint(double x, double y, double h) {
        Command X = new DriveToXPID(driveS,x,telemetry);
        Command Y = new DriveToYPID(driveS,y,telemetry);
//        Command H = new TurnToAnglePD(driveS,h,telemetry);
        return new SequentialCommandGroup(X,new WaitCommand(200),Y);
    }
}
