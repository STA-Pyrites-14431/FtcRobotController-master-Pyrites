package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoRedOne")
public class AutoRedOne extends LinearOpMode {

    BotDrive drive = new BotDrive();

    GoBildaPinpointDriver ODM;

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake
    DcMotor motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;

    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max;
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
        motorR = hardwareMap.get(DcMotor.class,"motorR"); //EH3

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        ODM.resetPosAndIMU();
        ODM.update();

        double heading = ODM.getHeading(AngleUnit.RADIANS);

//        drive.enableIntake(motorI,servoR1,servoR2,servoR3);
        drive.forward(motorFL,motorFR,motorBL,motorBR,1,400);
        drive.enableLaunch(motorLR,motorLL,0.45);
        Thread.sleep(2200);
        drive.enableRamp(motorR);
        Thread.sleep(1000);
        drive.disableRamp(motorR);
        Thread.sleep(250);
        drive.enableRamp(motorR);
        Thread.sleep(1000);
        drive.disableRamp(motorR);
        Thread.sleep(500);
        drive.turnLeft(motorFL,motorFR,motorBL,motorBR,1,115);
        Thread.sleep(500);
        drive.strafeRight(motorFL,motorFR,motorBL,motorBR,0.5,250);
        Thread.sleep(500);
        drive.turnRight(motorFL,motorFR,motorBL,motorBR,1,170);
//        drive.forward(motorFL,motorFR,motorBL,motorBR,1,500);
//        drive.forward(motorFL,motorFR,motorBL,motorBR,1,600);
//        drive.disableIntake(motorI,servoR1,servoR2,servoR3);
        ODM.update();

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
