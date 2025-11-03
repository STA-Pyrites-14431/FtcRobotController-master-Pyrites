package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoRedOdom")
public class AutoRedOdom extends LinearOpMode {

    BotDriveOdom odomDrive = new BotDriveOdom();

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

    double axial;
    double lateral;
    double yaw;
    double powerFL;
    double powerFR;
    double powerBL;
    double powerBR;
    double max;
    double ticks = 537.7;
    double ticksPerInch = 20.9;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.get(DcMotor.class,"motorFL"); //EH0
        motorFR = hardwareMap.get(DcMotor.class,"motorFR"); //CH0
        motorBL = hardwareMap.get(DcMotor.class,"motorBL"); //EH1
        motorBR = hardwareMap.get(DcMotor.class,"motorBR"); //CH1
        motorLR = hardwareMap.get(DcMotor.class,"motorLR"); //CH2
        motorLL = hardwareMap.get(DcMotor.class,"motorLL"); //EH2
        motorI = hardwareMap.get(DcMotor.class,"motorI"); //CH3
        servoR1 = hardwareMap.get(CRServo.class,"servoR1"); //EH0
        servoR2 = hardwareMap.get(CRServo.class,"servoR2"); //EH1
        servoR3 = hardwareMap.get(CRServo.class,"servoR3"); //EH2

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Hopefully move bot 1 foot forward, left, backwards, and right
        odomDrive.forward(motorFL,motorFR,motorBL,motorBR,0.6,12);
        Thread.sleep(500);
        odomDrive.strafeLeft(motorFL,motorFR,motorBL,motorBR,0.6,12);
        Thread.sleep(500);
        odomDrive.backward(motorFL,motorFR,motorBL,motorBR,0.6,12);
        Thread.sleep(500);
        odomDrive.strafeRight(motorFL,motorFR,motorBL,motorBR,0.6,12);
    }
}
