package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BotDriveEX {
    public void forward(MotorEx motorFL, MotorEx motorFR, MotorEx motorBL, MotorEx motorBR, double speed, long t) throws InterruptedException {
        motorFL.set(-speed);
        motorFR.set(-speed);
        motorBL.set(speed);
        motorBR.set(speed);

        Thread.sleep(t);

        motorFL.set(0);
        motorFR.set(0);
        motorBL.set(0);
        motorBR.set(0);

    }
    public void strafeOdom(MotorEx[] motors, GoBildaPinpointDriver ODM, int dy) {
        double speed;
        double posy = ODM.getPosY(DistanceUnit.INCH);
//        ODM.resetPosAndIMU();

        while (!(posy>dy-0.3 && posy<dy+0.3)) {
            if (dy < posy) {
                speed = -0.5;
            } else {
                speed = 0.5;
            }
            motors[0].set(speed);
            motors[1].set(-speed);
            motors[2].set(-speed);
            motors[3].set(speed);
            ODM.update();
            posy = ODM.getPosY(DistanceUnit.INCH);
        }
        motors[0].set(0);
        motors[1].set(0);
        motors[2].set(0);
        motors[3].set(0);
    }
    public void driveOdom(MotorEx[] motors, GoBildaPinpointDriver ODM, int dx) {
        double speed;
        double posx = ODM.getPosX(DistanceUnit.INCH);
//        ODM.resetPosAndIMU();

        while (!(posx>dx-0.3 && posx<dx+0.3)) {
            if (dx < posx) {
                speed = -0.5;
            } else {
                speed = 0.5;
            }
            motors[0].set(speed);
            motors[1].set(speed);
            motors[2].set(speed);
            motors[3].set(speed);
            ODM.update();
            posx = ODM.getPosX(DistanceUnit.INCH);
        }
        motors[0].set(0);
        motors[1].set(0);
        motors[2].set(0);
        motors[3].set(0);
    }
    public void turnOdom(MotorEx[] motors, GoBildaPinpointDriver ODM, double d) {
        d*=-1;
        double speed;
        double heading = ODM.getHeading(AngleUnit.DEGREES);

        while (!(heading>d-0.1 && heading<d+0.1)) {
            if (d > heading) {
                speed = -0.4;
            } else {
                speed = 0.4;
            }
            motors[0].set(speed);
            motors[1].set(-speed);
            motors[2].set(speed);
            motors[3].set(-speed);
            ODM.update();
            heading = ODM.getHeading(AngleUnit.DEGREES);
        }
        motors[0].set(0);
        motors[1].set(0);
        motors[2].set(0);
        motors[3].set(0);
    }
    public void enableLaunch(MotorEx motorLR, MotorEx motorLL, double power) throws InterruptedException {
        motorLR.set(power);
        motorLL.set(-power);
    }
    public void disableLaunch(MotorEx motorLR, MotorEx motorLL) throws InterruptedException {
        motorLR.set(0);
        motorLL.set(0);
    }
    public void enableIntake(MotorEx motorI) throws InterruptedException{
        motorI.set(1);
    }
    public void disableIntake(MotorEx motorI) throws InterruptedException{
        motorI.set(0);
    }
    public void enableRamp(MotorEx motorR) {
        motorR.set(0.4);
    }
    public void disableRamp(MotorEx motorR) {
        motorR.set(0);
    }
}
