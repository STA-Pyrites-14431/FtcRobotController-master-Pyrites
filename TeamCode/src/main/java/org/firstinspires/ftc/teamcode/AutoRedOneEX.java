package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "AutoRedOneEX")
public class AutoRedOneEX extends LinearOpMode {

    BotDriveEX driveEx = new BotDriveEX();
    GoBildaPinpointDriver ODM;

    MotorEx motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = new MotorEx(hardwareMap,"motorFL");
        motorFR = new MotorEx(hardwareMap,"motorFR");
        motorBL = new MotorEx(hardwareMap,"motorBL");
        motorBR = new MotorEx(hardwareMap,"motorBR");
        motorLL = new MotorEx(hardwareMap,"motorLL");
        motorLR = new MotorEx(hardwareMap,"motorLR");
        motorI = new MotorEx(hardwareMap,"motorI");
        motorR = new MotorEx(hardwareMap,"motorR");

        motorFL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorI.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorFR.setInverted(true);
        motorBL.setInverted(true);
        motorFL.setInverted(true);

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");
        ODM.resetPosAndIMU();
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        Pose2D start = new Pose2D(DistanceUnit.INCH,-60,24, AngleUnit.DEGREES,0);

        MotorEx[] motors = {motorFL, motorFR, motorBL, motorBR};
        double speed = 0.6;
        double closePower = 0.40;
        double farPower = 0.50;

        waitForStart();
        ODM.setPosition(start);
        while (opModeIsActive()) {
            driveEx.turnOdom(motors,ODM,90);
//            driveEx.driveOdom(motors,ODM,-48);
//            sleep(200);
//            driveEx.strafeOdom(motors,ODM,12);
//            sleep(200);
//            driveEx.driveOdom(motors,ODM,24);
//            sleep(200);
//            driveEx.turnOdom(motors,ODM,-135);
//            driveEx.enableLaunch(motorLR,motorLL,closePower);
//            sleep(1250);
//            driveEx.enableRamp(motorR);
//            driveEx.enableIntake(motorI);
//            sleep(2000);
//            driveEx.disableIntake(motorI);
//            driveEx.disableRamp(motorR);
//            driveEx.disableLaunch(motorLR,motorLL);
//            driveEx.turnOdom(motors,ODM,90);
//            telemetry.addData("XPos: ",ODM.getPosX(DistanceUnit.INCH));
//            telemetry.addData("YPos: ",ODM.getPosY(DistanceUnit.INCH));
//            telemetry.update();
//            driveEx.forward(motorFL,motorFR,motorBL,motorBR,0.5,500);
//            telemetry.addData("XPos: ",ODM.getPosX(DistanceUnit.INCH));
//            telemetry.addData("YPos: ",ODM.getPosY(DistanceUnit.INCH));
//            telemetry.update();
            break;
        }

    }
}
