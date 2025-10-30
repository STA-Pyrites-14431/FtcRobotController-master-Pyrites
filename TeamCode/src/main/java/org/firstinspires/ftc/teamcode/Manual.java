package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "manual")
public class Manual extends OpMode {

    DcMotor motorFL; //FrontLeft motor
    DcMotor motorFR; //FrontRight motor
    DcMotor motorBL; //BackLeft motor
    DcMotor motorBR; //BackRight motor
    DcMotor motorLL; //LauncherLeft motor
    DcMotor motorLR; //LauncherRight motor
    DcMotor motorI; //Intake motor
    CRServo servoR1; //Ramp1 servo
    CRServo servoR2; //Ramp2 servo
    CRServo servoR3; //Ramp3 servo

    float axial;
    float lateral;
    float yaw;
    float powerFL;
    float powerFR;
    float powerBL;
    float powerBR;
    float max;

    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class,"motorFL"); //EH0
        motorFR = hardwareMap.get(DcMotor.class,"motorFR"); //CH0
        motorBL = hardwareMap.get(DcMotor.class,"motorBL"); //EH1
        motorBR = hardwareMap.get(DcMotor.class,"motorBR"); //CH1
        motorLR = hardwareMap.get(DcMotor.class,"motorLR"); //CH2
        motorLL = hardwareMap.get(DcMotor.class,"motorLL"); //EH2
        motorI = hardwareMap.get(DcMotor.class,"motorI"); //CH3
        servoR1 = hardwareMap.get(CRServo.class,"servoR1"); //EH0
        servoR2 = hardwareMap.get(CRServo.class,"servoR2"); //EH1
        servoR3 = hardwareMap.get(CRServo.class,"servoR3"); //EH2
    }

    @Override
    public void loop() {

        //sets the direction that each wheel will spin
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        axial = -gamepad1.left_stick_y; //gets input of up and down of left stick for the forward and backwards robot driving //negative since up is negative for some reason
        lateral = gamepad1.left_stick_x; //gets input of left and right of left stick for the robot strafing
        yaw = gamepad1.right_stick_x; //gets input of left and right of right stick for the robot turning

        //calculates the power for each wheel based on how the three inputs would work together
        powerFL = axial - lateral + yaw;
        powerFR = axial - lateral - yaw;
        powerBL = axial + lateral + yaw;
        powerBR = axial + lateral - yaw;

        //calculates the max power of the wheels so that way none of them go over 100% or 1.0
        max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
        max = Math.max(max, Math.abs(powerBL));
        max = Math.max(max, Math.abs(powerBR));

        //if the max is over 100% or 1.0, lessen the power for the wheels
        if (max > 1.0) {
            powerFL /= max;
            powerFR /= max;
            powerBL /= max;
            powerBR /= max;
        }

        //sets the power for the wheels
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

       //show the input amount from each stick on driver hub
        telemetry.addData("Driving Input: ",axial);
        telemetry.addData("Strafing Input: ",lateral);
        telemetry.addData("Turning Input: ",yaw);

        //show the power of each wheel as a percentage on driver hub
        telemetry.addData("Front-Left Wheel Power: ",powerFL);
        telemetry.addData("Front-Right Wheel Power: ",powerFR);
        telemetry.addData("Back-Left Wheel Power: ",powerBL);
        telemetry.addData("Back-Right Wheel Power: ",powerBR);



        if (gamepad2.right_bumper || gamepad1.right_bumper) { //Turn on launcher
            motorLL.setPower(-0.40);
            motorLR.setPower(0.40);
        } else { //Turn off launcher
            motorLL.setPower(0);
            motorLR.setPower(0);
        }

        if (gamepad2.left_bumper|| gamepad1.left_bumper) {
            motorI.setPower(1);
        } else {
            motorI.setPower(0);
        }
        if (gamepad1.dpad_up) {
            servoR1.setPower(1);
            servoR2.setPower(1);
            servoR3.setPower(1);
        } else {
            servoR1.setPower(0);
            servoR2.setPower(0);
            servoR3.setPower(0);
        }


        //Strafe left. FL and BR spin backwards; FR and BL spin forwards
        //Strafe right. FR and BL spin backwards; FL and BR spin forwards
    }

}
