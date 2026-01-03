package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.function.InterruptableThrowingRunnable;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
    public void strafeOdom(DcMotor[] motors, GoBildaPinpointDriver ODM, int dy) {
        double speed;
        double posy = ODM.getPosY(DistanceUnit.INCH);
//        ODM.resetPosAndIMU();

        while (!(posy>dy-0.5 && posy<dy+0.5)) {
            if (dy < posy) {
                speed = -0.5;
            } else {
                speed = 0.5;
            }
            motors[0].setPower(speed);
            motors[1].setPower(-speed);
            motors[2].setPower(-speed);
            motors[3].setPower(speed);
            ODM.update();
            posy = ODM.getPosY(DistanceUnit.INCH);
        }
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);
    }
    public void driveOdom(DcMotor[] motors, GoBildaPinpointDriver ODM, int dx) {
        double speed;
        double posx = ODM.getPosX(DistanceUnit.INCH);
//        ODM.resetPosAndIMU();

        while (!(posx>dx-0.5 && posx<dx+0.5)) {
            if (dx < posx) {
                speed = -0.5;
            } else {
                speed = 0.5;
            }
            motors[0].setPower(speed);
            motors[1].setPower(speed);
            motors[2].setPower(speed);
            motors[3].setPower(speed);
            ODM.update();
            posx = ODM.getPosX(DistanceUnit.INCH);
        }
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);
    }
    public void turnOdom(DcMotor[] motors, GoBildaPinpointDriver ODM, double d) {
        d*=-1;
        double speed;
        double heading = ODM.getHeading(AngleUnit.DEGREES);

        while (!(heading>d-0.2 && heading<d+0.2)) {
            if (d > heading) {
                speed = -0.4;
            } else {
                speed = 0.4;
            }
            motors[0].setPower(speed);
            motors[1].setPower(-speed);
            motors[2].setPower(speed);
            motors[3].setPower(-speed);
            ODM.update();
            heading = ODM.getHeading(AngleUnit.DEGREES);
        }
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);
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
