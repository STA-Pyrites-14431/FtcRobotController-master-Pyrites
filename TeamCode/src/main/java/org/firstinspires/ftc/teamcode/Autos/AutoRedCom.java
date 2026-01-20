package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.DriveToX;
import org.firstinspires.ftc.teamcode.Commands.DriveToXPID;
import org.firstinspires.ftc.teamcode.Commands.DriveToY;
import org.firstinspires.ftc.teamcode.Commands.DriveToYPID;
import org.firstinspires.ftc.teamcode.Commands.TurnToAngle;
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
    Command lE, lD, rE, rD, iE, iD;
    Command x24PID, y24PID, t90PD;
    SequentialCommandGroup sqnc1, shoot, sqnc2, test;
    WaitCommand p;

    @Override
    public void initialize() {
        driveS = new Drive(hardwareMap, telemetry);
        intakeS = new Intake(hardwareMap);
        launcherS = new Launcher(hardwareMap);
        rampS = new Ramp(hardwareMap);

        //basic wait commands
        p = new WaitCommand(200);

        //turn commands
        t0 = new TurnToAngle(driveS, 0);
        t90 = new TurnToAngle(driveS, 90);
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

        //testing commands
        x24PID = new DriveToXPID(driveS,24, telemetry);
        y24PID = new DriveToYPID(driveS,24, telemetry);
        t90PD = new TurnToAnglePD(driveS,90, telemetry);

        //putting commands together
        sqnc1 = new SequentialCommandGroup(t90);
        shoot = new SequentialCommandGroup(lE,new WaitCommand(1250),rE,iE,new WaitCommand(2000),lD,rD,iD);
        sqnc2 = new SequentialCommandGroup(t0);
        test = new SequentialCommandGroup(t90,p,t0,p,x24,p,y24,p,x0,p,y0);

        schedule(test,new WaitCommand(200),shoot);
    }
}
