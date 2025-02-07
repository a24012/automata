

package org.firstinspires.ftc.robotcontroller;

/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name="Basic: Omni Linear OpMode", group="Linear OpMode")

public class AutoDavid2 extends LinearOpMode {

    //    Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MID = null;
    private DcMotor MIA = null;
    private DcMotor MDD = null;
    private DcMotor MDA = null;
    private ColorSensor sensorColor;
    //private Servo myServo=null;
    boolean hasExecuted = false;
    float[] hsv = {0.0f, 0.0f, 0.0f};
    float rojo = 0, verde = 0, azul = 0;

    private Servo SIM = null;
    private Servo SDM = null;
    private Servo SGO = null;

    float SCALE_FACTOR = 255;
    static final double TICKS_PER_REV = 537.6;
    static final double CIRCUNFERENCIA_CM = 14;// Circunferencia de la rueda en cm
    static final double TICKS_PER_CM = TICKS_PER_REV / CIRCUNFERENCIA_CM;

    @Override
    public void runOpMode() {

        sensorColor = hardwareMap.colorSensor.get("sensorColor");
        // Inicializar los motores
        SDM = hardwareMap.get(Servo.class, "SDM");
        SIM = hardwareMap.get(Servo.class, "SIM");
        SGO = hardwareMap.get(Servo.class, "SGO");
        MID = hardwareMap.get(DcMotor.class, "MID");
        MIA = hardwareMap.get(DcMotor.class, "MIA");
        MDD = hardwareMap.get(DcMotor.class, "MDD");
        MDA = hardwareMap.get(DcMotor.class, "MDA");

        // Resetear los encoders
        MID.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MIA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        runtime.reset();//ICV PREGUNTA SI SE DEBE LLEVAR CRONOMETRO PROPIO DE 30 SEGUNDOS

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //   1     P a s o
            encoderAvanza(1,1,70,1,1,-1,-1);
            //   2     P a s o
            encoderAvanza(2,1,30,-1,-1,1,1);
            //   3     P a s o
            encoderAvanza(3,1,70,1,-1,1,-1);
            //   4     P a s o
            encoderAvanza(4,1,65,1,1,-1,-1);
            //   5     P a s o
            encoderAvanza(5,1,70,1,-1,1,-1);
            //   6     P a s o
            encoderAvanza(6,1,62,1,1,1,1);
            //   7     P a s o
            encoderAvanza(7,1,130,-1,-1,1,1);
            //   8     P a s o
            encoderAvanza(8,1,140,1,1,-1,-1);
            //   9     P a s o
            encoderAvanza(9,1,20,-1,1,-1,1);
            //   10     P a s o
            encoderAvanza(10,1,140,-1,-1,1,1);

            //recoger
            SGO.setPosition(-0.30);
            sensorColor.enableLed(true);

            rojo = sensorColor.red();
            azul = sensorColor.blue();
            verde = sensorColor.green();

            Color.RGBToHSV((int) (rojo * SCALE_FACTOR), (int) (verde * SCALE_FACTOR), (int) (azul * SCALE_FACTOR), hsv);
            SDM.setPosition(0.285);
            SIM.setPosition(0.40);

            if (!hasExecuted && azul > 0.5) {
                SGO.setPosition(.36);
                hasExecuted = true;
                sensorColor.enableLed(false);

            }else{

                //   11 A LA IZQUIRDA
                encoderAvanza(11,1,15,1,-1,1,-1);
                //a la izquierda
                //   12 A LA DERECHA
                encoderAvanza(12,1,30,-1,1,-1,1);
                //a la derecha

                sensorColor.enableLed(true);
            }


        }
    }




    /*
     * funsion que raliza los movimientos
     *
     *numero de paso  speed pude ser algo parr controlar la velocidad
     *
     * */
    public void encoderAvanza(int paso,double speed, int moverCentimetos, int setPowerMID, int setPowerMIA,int setPowerMDD ,int setPowerMDA) {

        targetTicks = (int) (moverCentimetos * TICKS_PER_CM);

        // Configurar el objetivo de los encoders
        MID.setTargetPosition(targetTicks);
        MIA.setTargetPosition(targetTicks);
        MDD.setTargetPosition(targetTicks);
        MDA.setTargetPosition(targetTicks);

        // Cambiar el modo de los motores para correr hacia la posición
        MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Establecer la potencia del motor para avanzar
        MID.setPower(setPowerMID);
        MIA.setPower(setPowerMIA);
        MDD.setPower(setPowerMDD);
        MDA.setPower(setPowerMDA);

        // Esperar hasta que los motores lleguen a la posición
        while (opModeIsActive() && (MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy())) {
            telemetry.addData("Path", "Paso "+paso+" Avanzas"+moverCentimetos+"cm de %7d :%7d", targetTicks, MID.getCurrentPosition());
            telemetry.update();
        }

        // Detener todos los motores
        MID.setPower(0);
        MIA.setPower(0);
        MDD.setPower(0);
        MDA.setPower(0);

        // Cambiar el modo de los motores para usar el encoder
        MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path", "Completado");
        telemetry.update();

    }
}