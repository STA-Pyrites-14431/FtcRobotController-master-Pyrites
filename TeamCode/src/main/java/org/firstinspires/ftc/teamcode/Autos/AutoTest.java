package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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

public class AutoTest extends CommandOpMode {

    Drive driveS;
    Intake intakeS;
    Launcher launcherS;
    Ramp rampS;
    Command t0, t90, x24, y24, x0, y0;
    Command lE, lD, rE, rD, iE, iD;
    Command x24PID, y24PID, t90PD;
    SequentialCommandGroup test;
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
        x0 = new DriveToX(driveS,0);
        x24 = new DriveToX(driveS,24);
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
        x24PID = new DriveToXPID(driveS,24);
        y24PID = new DriveToYPID(driveS,24);
        t90PD = new TurnToAnglePD(driveS,90);

        //putting commands together

        test = new SequentialCommandGroup(x24PID);

        schedule(test);
    }
}
