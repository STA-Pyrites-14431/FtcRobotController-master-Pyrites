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
    CRServo servoPL; //PickupLeft motor
    CRServo servoPR; //PickupRight motor
    CRServo servoRL; //RampLeft servo
    CRServo  servoRR; //RampRight servo
    float lStickX;
    float lStickY;
    float rStickX;
    float rStickY;

    @Override
    public void init() {
        motorFL = hardwareMap.get(DcMotor.class,"motorFL"); //EH0
        motorFR = hardwareMap.get(DcMotor.class,"motorFR"); //CH0
        motorBL = hardwareMap.get(DcMotor.class,"motorBL"); //EH1
        motorBR = hardwareMap.get(DcMotor.class,"motorBR"); //CH1
        motorLR = hardwareMap.get(DcMotor.class,"motorLR"); //CH2
        motorLL = hardwareMap.get(DcMotor.class,"motorLL"); //EH2
        servoPL = hardwareMap.get(CRServo.class,"servoPL"); //CH0
        servoPR = hardwareMap.get(CRServo.class,"servoPR"); //EH0
        servoRL = hardwareMap.get(CRServo.class,"servoRL"); //CH1
        servoRR = hardwareMap.get(CRServo.class,"servoRR"); //EH1
    }

    @Override
    public void loop() {
        lStickY = gamepad1.left_stick_y;
        lStickX = gamepad1.left_stick_x;
//        rStickY = gamepad1.right_stick_y;
        rStickX = gamepad1.right_stick_x;
        servoPL.setPower(-1);
        servoPR.setPower(1);
        telemetry.addData("Left Stick Y:",lStickY);
        telemetry.addData("Left Stick X:",lStickX);
        telemetry.addData("Right Stick X: ",rStickX);
        if (lStickY > 0) { //Backwards
            motorFL.setPower(lStickY);
            motorFR.setPower(lStickY);
            motorBL.setPower(-lStickY);
            motorBR.setPower(-lStickY);
        } else if (lStickY < 0) { //Forwards
            motorFL.setPower(lStickY);
            motorFR.setPower(lStickY);
            motorBL.setPower(-lStickY);
            motorBR.setPower(-lStickY);
        } else if (lStickX > 0) {
            motorFL.setPower(lStickX);
            motorFR.setPower(lStickX);
            motorBL.setPower(lStickX);
            motorBR.setPower(lStickX);
        } else if (lStickX < 0) {
            motorFL.setPower(lStickX);
            motorFR.setPower(lStickX);
            motorBL.setPower(lStickX);
            motorBR.setPower(lStickX);
        } else if (rStickX > 0) { //Turn right
            motorFL.setPower(-rStickX);
            motorFR.setPower(rStickX);
            motorBL.setPower(rStickX);
            motorBR.setPower(-rStickX);
        } else if (rStickX < 0) { //Turn left
            motorFL.setPower(-rStickX);
            motorFR.setPower(rStickX);
            motorBL.setPower(rStickX);
            motorBR.setPower(-rStickX);
        } else { //Turn motor off
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
        if (gamepad1.right_bumper) { //Turn on launcher
            motorLL.setPower(-0.7);
            motorLR.setPower(0.7);
        } else { //Turn off launcher
            motorLL.setPower(0);
            motorLR.setPower(0);
        }


        //Strafe left. FL and BR spin backwards; FR and BL spin forwards
        //Strafe right. FR and BL spin backwards; FL and BR spin forwards
    }
}
