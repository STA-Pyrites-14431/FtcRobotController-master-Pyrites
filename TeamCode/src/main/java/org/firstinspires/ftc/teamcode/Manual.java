package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "manual")
public class Manual extends OpMode {

    DcMotor motorFL; //FrontLeft motor
    DcMotor motorFR; //FrontRight motor
    DcMotor motorBL; //BackLeft motor
    DcMotor motorBR; //BackRight motor
    DcMotor motorLL; //LauncherLeft motor
    DcMotor motorLR; //LauncherRight motor
    DcMotor motorPL; //PickupLeft motor
    DcMotor motorPR; //PickupRight motor
    CRServo servo; //Chain servo
    float lStickX;
    float lStickY;

    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class,"motorFL");
        motorFR = hardwareMap.get(DcMotor.class,"motorFR");
        motorBL = hardwareMap.get(DcMotor.class,"motorBL");
        motorBR = hardwareMap.get(DcMotor.class,"motorBR");
        motorLR = hardwareMap.get(DcMotor.class,"motorLR");
        motorLL = hardwareMap.get(DcMotor.class,"motorLL");
        motorPL = hardwareMap.get(DcMotor.class,"motorPL");
        motorPR = hardwareMap.get(DcMotor.class,"motorPR");
        servo = hardwareMap.get(CRServo.class,"servo");
    }

    @Override
    public void loop() {
        lStickY = gamepad1.left_stick_y;
        lStickX = gamepad1.left_stick_x;
        telemetry.addData("Left Stick Y:",lStickY);
        telemetry.addData("Left Stick X:",lStickX);
        if (lStickY > 0) {
            motorFL.setPower(-lStickY);
            motorFR.setPower(lStickY);
            motorBL.setPower(-lStickY);
            motorBR.setPower(lStickY);
        } else if (lStickY < 0) {
            motorFL.setPower(lStickY);
            motorFR.setPower(-lStickY);
            motorBL.setPower(lStickY);
            motorBR.setPower(-lStickY);
        } else if (lStickX > 0) {
            motorFL.setPower(-lStickX);
            motorFR.setPower(-lStickX);
            motorBL.setPower(-lStickX);
            motorBR.setPower(-lStickX);
        } else if (lStickX < 0) {
            motorFL.setPower(lStickX);
            motorFR.setPower(lStickX);
            motorBL.setPower(lStickX);
            motorBR.setPower(lStickX);
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }
}
