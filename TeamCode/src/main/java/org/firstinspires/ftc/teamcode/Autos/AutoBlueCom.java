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
    Command y0,y24,y48,t45,t90, tShoot;
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
        tShoot = new TurnToAnglePD(driveS,-122,telemetry);
        y0 = new DriveToYPID(driveS,0,telemetry);
        y24 = new DriveToYPID(driveS,-24,telemetry);
        y48 = new DriveToYPID(driveS,-48,telemetry);
        t90 = new TurnToAnglePD(driveS,-90,telemetry);

        Command x24 = DriveX(12);
        Command xn24 = DriveX(-12);

        double sH = -122; //shoot angle

        Command t0 = TurnH(122);
        Command tn90 = TurnH(-90);
        Command shoot0 = DriveXYTurnH(96,-12,sH);
        Command pickupPoint1 = DriveXYTurnH(-24,12,90);
        Command shoot1 = DriveXYTurnH(24,-12,sH);
        Command pickupPoint2 = DriveXYTurnH(-48,12,90);
        Command shoot2 = DriveXYTurnH(48,-12,sH);
        Command pickupPoint3 = DriveXYTurnH(-72,12,90);
        Command shoot3 = DriveXYTurnH(72,-12,sH);
        Command end = DriveXYTurnH(-36,12,90);


        shoot = new SequentialCommandGroup(lE,new WaitCommand(2000),rE,iE,new WaitCommand(3000),lD,rD,iD,t0);
        SequentialCommandGroup pickup1 = new SequentialCommandGroup(new InstantCommand(intakeS::forward),new WaitCommand(200),x24,new WaitCommand(200),xn24,new InstantCommand(intakeS::disable));
        SequentialCommandGroup pickup2 = new SequentialCommandGroup(new InstantCommand(intakeS::forward),new WaitCommand(200),x24,new WaitCommand(200),xn24,new InstantCommand(intakeS::disable));
        SequentialCommandGroup pickup3 = new SequentialCommandGroup(new InstantCommand(intakeS::forward),new WaitCommand(200),x24,new WaitCommand(200),xn24,new InstantCommand(intakeS::disable));

        SequentialCommandGroup initialShoot = new SequentialCommandGroup(shoot0,new WaitCommand(200),shoot);
        SequentialCommandGroup firstShoot = new SequentialCommandGroup(pickupPoint1,new WaitCommand(200),pickup1,new WaitCommand(200),TurnH(-90),new WaitCommand(200),shoot1,new WaitCommand(200),shoot);
        SequentialCommandGroup secondShoot = new SequentialCommandGroup(pickupPoint2,new WaitCommand(200),pickup2,new WaitCommand(200),TurnH(-90),new WaitCommand(200),shoot2,new WaitCommand(200),shoot);
        SequentialCommandGroup thirdShoot = new SequentialCommandGroup(pickupPoint3,new WaitCommand(200),pickup3,new WaitCommand(200),TurnH(-90),new WaitCommand(200),shoot3,new WaitCommand(200),shoot);

        schedule(new SequentialCommandGroup(initialShoot,w,firstShoot,w,secondShoot,w,thirdShoot,w,end));
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
