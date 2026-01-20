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
    Command tS;
    Command lE, lD, rE, rD, iE, iD;
    Command x24PID, y24PID, t90PD;
    SequentialCommandGroup shoot, test;
    SequentialCommandGroup p1;
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

        //putting commands together
        p1 = new SequentialCommandGroup(x24,w,y24);
        shoot = new SequentialCommandGroup(lE,new WaitCommand(1250),rE,iE,new WaitCommand(2000),lD,rD,iD);
        test = new SequentialCommandGroup(t90,w,t0,w,x24,w,y24,w,x0,w,y0);

        schedule(test,new WaitCommand(200),shoot);
    }
}
