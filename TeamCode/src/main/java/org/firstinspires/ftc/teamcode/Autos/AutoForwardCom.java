package org.firstinspires.ftc.teamcode.Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.DriveToXPID;
import org.firstinspires.ftc.teamcode.Commands.TurnToAnglePD;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class AutoForwardCom extends CommandOpMode {

    Drive driveS;
    Command xn48, t90;
    @Override
    public void initialize() {
        driveS = new Drive(hardwareMap,telemetry);
        xn48 = new DriveToXPID(driveS,-48,telemetry);
        t90 = new TurnToAnglePD(driveS,90,telemetry);
        schedule(new SequentialCommandGroup(xn48,t90));
    }
}
