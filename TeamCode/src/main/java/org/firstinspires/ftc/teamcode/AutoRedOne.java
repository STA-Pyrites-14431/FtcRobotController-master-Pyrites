package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "AutoRedOne")
public class AutoRedOne extends LinearOpMode {

    BotDrive drive = new BotDrive();

    GoBildaPinpointDriver ODM;

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake
    DcMotor motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;


    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max;
    double speed = 1.0;

//    Pose2D position;
//    Pose2D p1, p2;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.get(DcMotor.class,"motorFL"); //EH0
        motorFR = hardwareMap.get(DcMotor.class,"motorFR"); //CH0
        motorBL = hardwareMap.get(DcMotor.class,"motorBL"); //EH1
        motorBR = hardwareMap.get(DcMotor.class,"motorBR"); //CH1
        motorLR = hardwareMap.get(DcMotor.class,"motorLR"); //CH2
        motorLL = hardwareMap.get(DcMotor.class,"motorLL"); //EH2
        motorI = hardwareMap.get(DcMotor.class,"motorI"); //CH3
        motorR = hardwareMap.get(DcMotor.class,"motorR"); //EH3\

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        ODM = hardwareMap.get(GoBildaPinpointDriver.class,"ODM");
        ODM.resetPosAndIMU();

//        p1 = new Pose2D(DistanceUnit.INCH,36,0,AngleUnit.RADIANS,Math.PI/4);
//        p2 = new Pose2D(DistanceUnit.INCH,12,12,AngleUnit.RADIANS,Math.PI/2);
        DcMotor[] motors = {motorFL, motorFR, motorBL, motorBR};
        double heading = ODM.getHeading(AngleUnit.RADIANS);
        double speed = 0.6;

        waitForStart();
        while (opModeIsActive()) {
//            telemetry.addData("Heading: ",ODM.getHeading(AngleUnit.DEGREES));
//            telemetry.update();
//            turn(motors,90);
//            telemetry.addData("Heading: ",ODM.getHeading(AngleUnit.DEGREES));
//            telemetry.update();
//            sleep(1000);
//            turn(motors, -90);
//            telemetry.addData("Heading: ",ODM.getHeading(AngleUnit.DEGREES));
//            telemetry.update();
//            sleep(10000);
            telemetry.addData("Heading: ",ODM.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            drive.turnOdom(motors,ODM,90);
            telemetry.addData("Heading: ",ODM.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            sleep(2000);
            drive.turnOdom(motors,ODM,0);
            telemetry.addData("Heading: ",ODM.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            break;
        }
    }
    public void turn(DcMotor[] motors, long d) throws InterruptedException{
        double speed = 0.6;
        if (d < 0) speed *= -1;
        d = Math.abs(d);
        double t = -(4.28763*Math.pow(10,-8))*Math.pow(d,4) + 0.0000451483*Math.pow(d,3) - 0.0155514*Math.pow(d,2) + 9.26416*d - 75.36675;

        motors[0].setPower(speed);
        motors[1].setPower(-speed);
        motors[2].setPower(speed);
        motors[3].setPower(-speed);
        ODM.update();

        Thread.sleep((long)t);

        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);
        ODM.update();
    }
}
