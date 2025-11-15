package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingRunnable;

public class BotDriveOdom {
    double ticksPerInch = 20.9;
    double newTarget;
    public void enableIntake(DcMotor motorI, DcMotor motorR) throws InterruptedException{
        motorI.setPower(1);
        motorR.setPower(-1);
    }
    public void disableIntake(DcMotor motorI, DcMotor motorR) throws InterruptedException{
        motorI.setPower(0);
        motorR.setPower(0);
    }
    public void forward(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, double inches) throws InterruptedException {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newTarget = ticksPerInch*inches;

        motorFL.setTargetPosition((int)newTarget);
        motorFR.setTargetPosition((int)newTarget);
        motorBL.setTargetPosition((int)newTarget);
        motorBR.setTargetPosition((int)newTarget);

        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void backward(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, double inches) throws InterruptedException{
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newTarget = ticksPerInch*inches;

        motorFL.setTargetPosition((int)newTarget);
        motorFR.setTargetPosition((int)newTarget);
        motorBL.setTargetPosition((int)newTarget);
        motorBR.setTargetPosition((int)newTarget);

        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void turnLeft(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long d) throws InterruptedException{

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
    public void turnRight(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long d) throws InterruptedException{

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
    public void strafeRight(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, double inches) throws InterruptedException{
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newTarget = ticksPerInch*inches;

        motorFL.setTargetPosition((int)newTarget);
        motorFR.setTargetPosition((int)newTarget);
        motorBL.setTargetPosition((int)newTarget);
        motorBR.setTargetPosition((int)newTarget);

        motorFL.setPower(-speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(-speed);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void strafeLeft(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, double inches) throws InterruptedException{
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newTarget = ticksPerInch*inches;

        motorFL.setTargetPosition((int)newTarget);
        motorFR.setTargetPosition((int)newTarget);
        motorBL.setTargetPosition((int)newTarget);
        motorBR.setTargetPosition((int)newTarget);

        motorFL.setPower(speed);
        motorFR.setPower(-speed);
        motorBL.setPower(-speed);
        motorBR.setPower(speed);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void launch(DcMotor motorLR, DcMotor motorLL, long t) throws InterruptedException {
        motorLR.setPower(0.4);
        motorLL.setPower(-0.4);

        Thread.sleep(t);

        motorLR.setPower(0);
        motorLL.setPower(0);
    }
}
