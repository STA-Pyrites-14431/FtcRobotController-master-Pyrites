package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// field size: 144in x 144in

@Autonomous(name = "AutoRedOdom")
public class AutoRedOdom extends LinearOpMode {

    GoBildaPinpointDriver odo;
    BotDriveOdom odomDrive = new BotDriveOdom();

    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake
    //FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Right
    //LL = Launcher Left, LR = Launcher Right, I = Intake, R = Ramp
    DcMotor motorFL, motorFR, motorBL, motorBR, motorLL, motorLR, motorI, motorR;

    double axial, lateral, yaw, powerFL, powerFR, powerBL, powerBR, max;
    int posFL=0, posFR=0, posBL=0, posBR=0;
    double ticks = 537.7;
    double ticksPerInch = 20.9;
    double newTarget;

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

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        newTarget = ticksPerInch*36;

        forward(0.6,36);
    }
    public void forward(double speed, double inches) throws InterruptedException {

        double target = ticksPerInch*inches;

        motorFL.setTargetPosition((int)target);
        motorFR.setTargetPosition((int)target);
        motorBL.setTargetPosition((int)target);
        motorBR.setTargetPosition((int)target);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);

        while (opModeIsActive() && motorFL.isBusy() && motorFR.isBusy() && motorBL.isBusy() && motorBR.isBusy()) {
            idle();
        }
    }
}
