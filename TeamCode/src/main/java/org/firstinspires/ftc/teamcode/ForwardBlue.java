package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ForwardBlue")
public class ForwardBlue extends LinearOpMode {

    BotDrive drive = new BotDrive();

    DcMotor motorFL; //FrontLeft motor
    DcMotor motorFR; //FrontRight motor
    DcMotor motorBL; //BackLeft motor
    DcMotor motorBR; //BackRight motor
    DcMotor motorLL; //LauncherLeft motor
    DcMotor motorLR; //LauncherRight motor
    DcMotor motorI; //Intake motor
    CRServo servoR1; //Ramp1 servo
    CRServo servoR2; //Ramp2 servo
    CRServo servoR3; //Ramp3 servo

    float axial;
    float lateral;
    float yaw;
    float powerFL;
    float powerFR;
    float powerBL;
    float powerBR;
    float max;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.get(DcMotor.class,"motorFL"); //EH0
        motorFR = hardwareMap.get(DcMotor.class,"motorFR"); //CH0
        motorBL = hardwareMap.get(DcMotor.class,"motorBL"); //EH1
        motorBR = hardwareMap.get(DcMotor.class,"motorBR"); //CH1

        waitForStart();

//        drive.enableIntake(motorI,servoR1,servoR2,servoR3);
        drive.strafeLeft(motorFL,motorFR,motorBL,motorBR,1,400);

    }
}
