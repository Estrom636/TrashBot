#include <ESP32Servo.h>

#include <Bluepad32.h>
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

Servo light;

Servo motorL;
double lPower = 0;
Servo motorR;
double rPower = 0;

bool armed = false;

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	light.setPeriodHertz(50);
  motorL.setPeriodHertz(50);
  motorR.setPeriodHertz(50);

  light.attach(33, 1000, 2500);
  motorL.attach(26, 1000, 2000);
  motorR.attach(27, 1000, 2000);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
}

void loop() {
  BP32.update();
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
    }
  }

  Serial.printf("ARMED: %s L: %.2f R: %.2f \n", armed ? "TRUE" : "FALSE", lPower, rPower);

  if( ((int) (millis()/1000)) % 2 == 0 && armed) {
    light.write(30);
  }else {
    light.write(0);
  }

  if(armed) {
    MotorPowL(lPower);
    MotorPowR(rPower);
  }else {
    MotorPowL(0);
    MotorPowR(0);
  }
}


void MotorPowL(double power) {
  if(power >  1) power =  1;
  if(power < -1) power = -1;

  motorL.write(90 + (power * 90));
}
void MotorPowR(double power) {
  if(power >  1) power =  1;
  if(power < -1) power = -1;

  motorR.write(90 + (power * -90));
}


void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;
  armed = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr gamepad) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

  if (gamepad->miscStart()) {
    armed = true;
    gamepad->setRumble(100, 75);
  }

  if (gamepad->miscSelect()) {
    armed = false;
  }

  lPower = gamepad->axisY()  / -512.0;
  rPower = gamepad->axisRY() / -512.0;

  // Another way to query the buttons, is by calling buttons(), or
  // miscButtons() which return a bitmask.
  // Some gamepads also have DPAD, axis and more.
  char buf[256];
  snprintf(buf, sizeof(buf) - 1,
           "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
           "axis L: %4li, %4li, axis R: %4li, %4li, "
           "brake: %4ld, throttle: %4li, misc: 0x%02x ",
           gamepad->index(),        // Gamepad Index
           gamepad->dpad(),         // DPad
           gamepad->buttons(),      // bitmask of pressed buttons
           gamepad->axisX(),        // (-511 - 512) left X Axis
           gamepad->axisY(),        // (-511 - 512) left Y axis
           gamepad->axisRX(),       // (-511 - 512) right X axis
           gamepad->axisRY(),       // (-511 - 512) right Y axis
           gamepad->brake(),        // (0 - 1023): brake button
           gamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
           gamepad->miscButtons()  // bitmask of pressed "misc" buttons
  );
  Serial.println(buf);
  // You can query the axis and other properties as well. See
  // Controller.h For all the available functions.
}