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
        motorLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        ODM.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        Pose2D start = new Pose2D(DistanceUnit.INCH,-60,24, AngleUnit.DEGREES,0);

        DcMotor[] motors = {motorFL, motorFR, motorBL, motorBR};
        double speed = 0.6;
        double closePower = 0.40;
        double farPower = 0.50;

        waitForStart();
        ODM.setPosition(start);
        while (opModeIsActive()) {
            drive.driveOdom(motors,ODM,-48);
            sleep(200);
//            resetODM();
            drive.strafeOdom(motors,ODM,12);
            sleep(200);
//            resetODM();
            drive.driveOdom(motors,ODM,24);
            sleep(200);
//            resetODM();
            drive.turnOdom(motors,ODM,-135);
            drive.enableLaunch(motorLR,motorLL,closePower);
            sleep(1250);
            drive.enableRamp(motorR);
            drive.enableIntake(motorI);
            sleep(2000);
            drive.disableIntake(motorI);
            drive.disableRamp(motorR);
            drive.disableLaunch(motorLR,motorLL);
            drive.turnOdom(motors,ODM,90);
//            resetODM();
//            drive.driveOdom(motors,ODM,12);
//            sleep(200);
////            resetODM();
//            drive.driveOdom(motors,ODM,-12);

            telemetry.addData("X Pos: ",ODM.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y Pos: ",ODM.getPosY(DistanceUnit.INCH));
            telemetry.update();
//            sleep(5000);
            break;
        }
    }
    public void resetODM() {
        ODM.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0));
    }
}
