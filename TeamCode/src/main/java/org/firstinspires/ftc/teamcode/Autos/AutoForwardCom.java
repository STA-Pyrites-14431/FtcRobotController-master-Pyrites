package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.DriveToXPID;
import org.firstinspires.ftc.teamcode.Commands.TurnToAnglePD;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class AutoForwardCom extends CommandOpMode {

    Drive driveS;
    Command x48, t90;
    @Override
    public void initialize() {
        driveS = new Drive(hardwareMap,telemetry);
        x48 = new DriveToXPID(driveS,12,telemetry);
        t90 = new TurnToAnglePD(driveS,90,telemetry);
        schedule(new SequentialCommandGroup(x48,t90));
    }
}
