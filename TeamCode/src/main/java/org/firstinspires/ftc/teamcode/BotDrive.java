package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingRunnable;

public class BotDrive {
    public void forward(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long t) throws InterruptedException {
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
    public void backward(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long t) throws InterruptedException{
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
    public void turnLeft(DcMotor motors[], long t) throws InterruptedException{

        motors[0].setPower(1);
        motors[1].setPower(-1);
        motors[2].setPower(-1);
        motors[3].setPower(1);

        Thread.sleep(t);

        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);
    }
    public void turnRight(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long t) throws InterruptedException{

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
    public void strafeRight(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long t) throws InterruptedException{
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void strafeLeft(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double speed, long t) throws InterruptedException{
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);

        Thread.sleep(t);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public void enableLaunch(DcMotor motorLR, DcMotor motorLL, double power) throws InterruptedException {
        motorLR.setPower(power);
        motorLL.setPower(-power);
    }
    public void disableLaunch(DcMotor motorLR, DcMotor motorLL) throws InterruptedException {
        motorLR.setPower(0);
        motorLL.setPower(0);
    }
    public void enableIntake(DcMotor motorI) throws InterruptedException{
        motorI.setPower(1);
    }
    public void disableIntake(DcMotor motorI) throws InterruptedException{
        motorI.setPower(0);
    }
    public void enableRamp(DcMotor motorR) {
        motorR.setPower(0.4);
    }
    public void disableRamp(DcMotor motorR) {
        motorR.setPower(0);
    }
}
