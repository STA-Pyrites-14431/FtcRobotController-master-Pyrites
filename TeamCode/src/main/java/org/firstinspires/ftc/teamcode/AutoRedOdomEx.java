package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// field size: 144in x 144in

//@Autonomous(name = "AutoRedOdomEx")
public class AutoRedOdomEx extends LinearOpMode {

    GoBildaPinpointDriver ODM;

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake, R = Ramp
    MotorEx motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;
    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max;
    MecanumDrive mec;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = new MotorEx(hardwareMap,"motorFL"); //EH0
        motorFR = new MotorEx(hardwareMap,"motorFR"); //CH0
        motorBL = new MotorEx(hardwareMap,"motorBL"); //EH1
        motorBR = new MotorEx(hardwareMap,"motorBR"); //CH1
        motorLR = new MotorEx(hardwareMap,"motorLR"); //CH2
        motorLL = new MotorEx(hardwareMap,"motorLL"); //EH2
        motorI = new MotorEx(hardwareMap,"motorI"); //CH3
        motorR = new MotorEx(hardwareMap,"motorR"); //EH3

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");

        ODM.setOffsets(0,0, DistanceUnit.INCH);
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        ODM.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ODM.resetPosAndIMU();
        Pose2D startingPos = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);
        ODM.setPosition(startingPos);

        mec = new MecanumDrive(motorFL, motorFR, motorBL, motorBR);

        motorFL.setRunMode(Motor.RunMode.VelocityControl);
        motorFR.setRunMode(Motor.RunMode.VelocityControl);
        motorBL.setRunMode(Motor.RunMode.VelocityControl);
        motorBR.setRunMode(Motor.RunMode.VelocityControl);

        motorFL.setInverted(true);
        motorFR.setInverted(true);

        waitForStart();

        motorFL.setVelocity(1);
        motorFR.setVelocity(1);
        motorBL.setVelocity(1);
        motorBR.setVelocity(1);

        sleep(250);

        motorFL.setVelocity(0);
        motorFR.setVelocity(0);
        motorBL.setVelocity(0);
        motorBR.setVelocity(0);
    }
}
