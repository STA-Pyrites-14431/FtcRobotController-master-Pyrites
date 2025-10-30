package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

public class BotDrive {
    public void enableIntake(DcMotor motorI, CRServo servoR1, CRServo servoR2, CRServo servoR3) throws InterruptedException{
        motorI.setPower(1);
        servoR1.setPower(1);
        servoR2.setPower(1);
        servoR3.setPower(1);
    }
    public void disableIntake(DcMotor motorI, CRServo servoR1, CRServo servoR2, CRServo servoR3) throws InterruptedException{
        motorI.setPower(0);
        servoR1.setPower(0);
        servoR2.setPower(0);
        servoR3.setPower(0);
    }
    public void forward(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, float speed, long t) throws InterruptedException {
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

    }
    public void backward(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, float speed, long t) throws InterruptedException{
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void turnLeft(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, float speed, long d) throws InterruptedException{

        long t = (long)((-(8.89092*Math.pow(10,-8))*Math.pow(d,4))+(0.000038866*Math.pow(d,3))+(0.00133745*Math.pow(d,2))+(3.62963*d)+(4.84676*Math.pow(10,-27)));
        motorFL.setPower(speed);
        motorFR.setPower(-speed);
        motorBL.setPower(-speed);
        motorBR.setPower(speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void turnRight(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, float speed, long d) throws InterruptedException{

        long t = (long)((-(8.89092*Math.pow(10,-8))*Math.pow(d,4))+(0.000038866*Math.pow(d,3))+(0.00133745*Math.pow(d,2))+(3.62963*d)+(4.84676*Math.pow(10,-27)));

        motorFL.setPower(-speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(-speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void strafeRight(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, float speed, long t) throws InterruptedException{
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void strafeLeft(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, float speed, long t) throws InterruptedException{
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
