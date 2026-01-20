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

@Autonomous(name = "AutoTest")
public class AutoTest extends CommandOpMode {

    Drive driveS;
    Intake intakeS;
    Launcher launcherS;
    Ramp rampS;
    Command lE, lD, rE, rD, iE, iD;
    Command x0,x24,x48,y0,y24,y48,t0,t45,t90;
    SequentialCommandGroup test;
    WaitCommand p;

    @Override
    public void initialize() {
        driveS = new Drive(hardwareMap, telemetry);
        intakeS = new Intake(hardwareMap);
        launcherS = new Launcher(hardwareMap);
        rampS = new Ramp(hardwareMap);

        p = new WaitCommand(500);

        //testing commands


        //putting commands together
        x0 = new DriveToXPID(driveS,0,telemetry);
        x24 = new DriveToXPID(driveS,24,telemetry);
        x48 = new DriveToXPID(driveS,48,telemetry);
        y0 = new DriveToYPID(driveS,0,telemetry);
        y24 = new DriveToYPID(driveS,24,telemetry);
        y48 = new DriveToYPID(driveS,48,telemetry);
        t0 = new TurnToAnglePD(driveS,0,telemetry);
        t45 = new TurnToAnglePD(driveS,45,telemetry);
        t90 = new TurnToAnglePD(driveS,90,telemetry);
        test = new SequentialCommandGroup(x24,p,y24,p,x0,p,y0,x48,t90,y48,t0,x0,y0);

        schedule(test);
    }
}
