package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// field size: 144in x 144in

@Autonomous(name = "AutoRedOdomEx")
public class AutoRedOdomEx extends LinearOpMode {

    GoBildaPinpointDriver ODM;

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake, R = Ramp
    MotorEx motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;
    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max;
    MecanumDrive mec;
    Pose2D position;
    Pose2D p1, p2;

    Waypoint w0, w1, w2;


    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = new MotorEx(hardwareMap,"motorFL",Motor.GoBILDA.RPM_312); //EH0
        motorFR = new MotorEx(hardwareMap,"motorFR",Motor.GoBILDA.RPM_312); //CH0
        motorBL = new MotorEx(hardwareMap,"motorBL",Motor.GoBILDA.RPM_312); //EH1
        motorBR = new MotorEx(hardwareMap,"motorBR",Motor.GoBILDA.RPM_312); //CH1
        motorLR = new MotorEx(hardwareMap,"motorLR"); //CH2
        motorLL = new MotorEx(hardwareMap,"motorLL"); //EH2
        motorI = new MotorEx(hardwareMap,"motorI",Motor.GoBILDA.RPM_223); //CH3
        motorR = new MotorEx(hardwareMap,"motorR",Motor.GoBILDA.RPM_312); //EH3

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");

        ODM.setOffsets(0,0, DistanceUnit.MM);
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        ODM.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ODM.resetPosAndIMU();
        Pose2D startingPos = new Pose2D(DistanceUnit.MM,0,0, AngleUnit.RADIANS,0);
        ODM.setPosition(startingPos);

        mec = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);

        position = ODM.getPosition();
        p1 = new Pose2D(DistanceUnit.MM,100,0,AngleUnit.RADIANS,45);
        p2 = new Pose2D(DistanceUnit.MM,50,50,AngleUnit.RADIANS,90);

//        motorFL.setRunMode(Motor.RunMode.VelocityControl);
//        motorFR.setRunMode(Motor.RunMode.VelocityControl);
//        motorBL.setRunMode(Motor.RunMode.VelocityControl);
//        motorBR.setRunMode(Motor.RunMode.VelocityControl);

        waitForStart();

        while (opModeIsActive()) {
            if (!goToPosition(p1)) {
                continue;
            }
            if (!goToPosition(p2)) {
                continue;
            }
            break;
        }

    }

    public boolean goToPosition(Pose2D destination) {
        Pose2D current = ODM.getPosition();
        double dx, dy;
        double xPower, yPower;

        double kLinear = 0.05;
        double kAngular = 0.5;
        double positionThreshold = 0.5;
        double headingThreshold = Math.toRadians(2);

        dx = destination.getX(DistanceUnit.MM) - current.getX(DistanceUnit.MM);
        dy = destination.getY(DistanceUnit.MM) - current.getY(DistanceUnit.MM);
        double distance = Math.hypot(dx, dy);

        xPower = kLinear * dx;
        yPower = kLinear * dy;

        double headingError = destination.getHeading(AngleUnit.RADIANS) - current.getHeading(AngleUnit.RADIANS);
        headingError = Math.atan2(Math.sin(headingError),Math.cos(headingError));
        double yawPower = kAngular * headingError;

        double max = Math.max(Math.abs(xPower),Math.max(Math.abs(yPower),Math.abs(yawPower)));
        if (max > 1.0) {
            xPower /= max;
            yPower /= max;
            yawPower /= max;
        }
        double gyroAngle = ODM.getHeading(AngleUnit.RADIANS);
        mec.driveFieldCentric(xPower,yPower,yawPower,gyroAngle);

        boolean atTarget = (distance < positionThreshold) && (Math.abs(headingError) < headingThreshold);

        if (distance < positionThreshold && Math.abs(headingError) < headingThreshold) {
            mec.driveFieldCentric(0,0,0,gyroAngle);
        }

        return atTarget;
    }
}
