package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutoRedOne")
public class AutoRedOne extends LinearOpMode {

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
    double speed = 1.0;

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

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

//        drive.enableIntake(motorI,servoR1,servoR2,servoR3);
        drive.forward(motorFL,motorFR,motorBL,motorBR,1,600);
        drive.enableIntake(motorI,servoR1,servoR2,servoR3);
        drive.launch(motorLR,motorLL,10000);
        drive.disableIntake(motorI,servoR1,servoR2,servoR3);
        drive.turnLeft(motorFL,motorFR,motorBL,motorBR,1,90);
        drive.forward(motorFL,motorFR,motorBL,motorBR,1,500);
//        drive.forward(motorFL,motorFR,motorBL,motorBR,1,600);
//        drive.disableIntake(motorI,servoR1,servoR2,servoR3);

    }

    public void PPG() {

    }
    public void PGP() {

    }
    public void GPP() {

    }
//    public void enableIntake() throws InterruptedException{
//        motorI.setPower(1);
//        servoR1.setPower(1);
//        servoR2.setPower(1);
//        servoR3.setPower(1);
//    }
//    public void disableIntake() throws InterruptedException{
//        motorI.setPower(0);
//        servoR1.setPower(0);
//        servoR2.setPower(0);
//        servoR3.setPower(0);
//    }
//    public void forward(float speed, long t) throws InterruptedException {
//        motorFL.setPower(-speed);
//        motorFR.setPower(-speed);
//        motorBL.setPower(speed);
//        motorBR.setPower(speed);
//
//        Thread.sleep(t);
//
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//
//    }
//    public void backward(float speed, long t) throws InterruptedException{
//        motorFL.setPower(speed);
//        motorFR.setPower(speed);
//        motorBL.setPower(-speed);
//        motorBR.setPower(-speed);
//
//        Thread.sleep(t);
//
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//    }
//    public void turnRight(float speed, long t) throws InterruptedException{
//        motorFL.setPower(speed);
//        motorFR.setPower(-speed);
//        motorBL.setPower(-speed);
//        motorBR.setPower(speed);
//
//        Thread.sleep(t);
//
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//    }
//    public void turnLeft(float speed, long t) throws InterruptedException{
//        motorFL.setPower(-speed);
//        motorFR.setPower(speed);
//        motorBL.setPower(speed);
//        motorBR.setPower(-speed);
//
//        Thread.sleep(t);
//
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//    }
//    public void strafeRight(float speed, long t) throws InterruptedException{
//        motorFL.setPower(-speed);
//        motorFR.setPower(-speed);
//        motorBL.setPower(speed);
//        motorBR.setPower(speed);
//
//        Thread.sleep(t);
//
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//    }
//    public void strafeLeft(float speed, long t) throws InterruptedException{
//        motorFL.setPower(-speed);
//        motorFR.setPower(-speed);
//        motorBL.setPower(speed);
//        motorBR.setPower(speed);
//
//        Thread.sleep(t);
//
//        motorFL.setPower(0);
//        motorFR.setPower(0);
//        motorBL.setPower(0);
//        motorBR.setPower(0);
//    }
}
