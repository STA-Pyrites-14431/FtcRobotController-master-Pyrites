package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="dataCollect")
public class DataCollect extends OpMode {

    BotDrive drive = new BotDrive();

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

    float speed = 0;
    int degrees = 0;
    int time = 0;

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
        if (gamepad1.dpad_up) {
            degrees += 5;
        } else if (gamepad1.dpad_down) {
            degrees -= 5;
        }
        if (gamepad1.dpad_left) {
            speed -= 0.1;
        } else if (gamepad1.dpad_right) {
            speed += 0.1;
        }
        if (gamepad1.right_bumper) {
            time += 100;
        } else if (gamepad1.left_bumper) {
            time -= 100;
        }
        telemetry.addData("Degrees",degrees);
        telemetry.addData("Speed",speed);
        telemetry.addData("Time",time);

    }
}
