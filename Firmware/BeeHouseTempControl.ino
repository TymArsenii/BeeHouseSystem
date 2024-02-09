/*
Sens1 - 2;
Sens2 - 3;
Relay1 - 4; // OUTPUT
Relay2 - 5;// OUTPUT
ButtonRed(down) - 6; // OUTPUT
ButtonGreen(down) - 7; // OUTPUT
Buzzer - 8; // OUTPUT

5 OUTPUT's
*/

//defaults ->
#define HOUSES_AMOUNT 1
#define HYSTERESYS 0.2

const uint8_t addr[][8] PROGMEM = {
    {0x28, 0x61, 0x64, 0x9, 0x55, 0xC7, 0xE4, 0x6B},
    {0x28, 0x61, 0x64, 0x9, 0x55, 0xC7, 0xFF, 0xD6},
};

float target_temps_arr[10] = {15, 15, 15};
//<- defaults

byte arrow_up[] = {
    B00000,
    B00100,
    B01110,
    B10101,
    B00100,
    B00100,
    B00100,
    B00000,
};

byte arrow_down[] = {
    B00000,
    B00100,
    B00100,
    B00100,
    B10101,
    B01110,
    B00100,
    B00000,
};

byte check[] = {B00000, B00000, B00001, B00010, B10100, B01000, B00000, B00000};

uint32_t sensor_read_timer;
uint32_t couter_timer;
uint32_t startup_timer;
uint32_t last_click_timer;
float temps_arr[10];
float temps_prev_arr[10];
uint8_t current_id;
bool force_update;
bool clicked;
bool fatal;
bool state;
bool state_prev;
int time_spent_h;
int time_spent_m;
int time_spent_s;
bool arrow_temp[10];

#include <microDS18B20.h>
MicroDS18B20<2, DS_ADDR_MODE, 2, DS_PROGMEM> sensors;

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

#include <EncButton.h>
Button up(7);
Button down(6);

#include <EEPROM.h>
#include <GyverTimers.h>
#include <PinChangeInterrupt.h>

#define EEPROM_KEY 3

void setup()
{
  Serial.begin(115200);

  lcd.createChar(1, arrow_up);
  lcd.createChar(0, arrow_down);
  lcd.createChar(2, check);

  Timer1.setFrequency(1);
  Timer1.enableISR();

  sensors.setAddress((uint8_t *)addr);
  sensors.setResolutionAll(12);
  sensors.requestTempAll();

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(4, 1);
  digitalWrite(5, 1);
  digitalWrite(10, 0);

  if (EEPROM[0] != EEPROM_KEY)
  {
    for (int id = 1; id < 600; id++)
    {
      EEPROM.put(id, 0);
    }
    EEPROM[0] = EEPROM_KEY;
    EEPROM.put(5, 15.0);
  }

  EEPROM.get(5, target_temps_arr[1]);

  lcd.init();
  lcd.backlight();

  up.setStepTimeout(80);
  down.setStepTimeout(80);

  up.setHoldTimeout(800);
  down.setHoldTimeout(800);

  lcd.setCursor(5, 1);
  lcd.print("Bee Houses");
  lcd.setCursor(4, 2);
  lcd.print("Temp Control");
  delay(1000);

  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Bee Houses");
  lcd.setCursor(5, 2);
  lcd.print("by Arsenii");
  //delay(1000);
  startup_timer = millis();
  attachPCINT(digitalPinToPCINT(7), up_tick, FALLING);
  attachPCINT(digitalPinToPCINT(6), down_tick, FALLING);
  lcd.clear();

  lcd.setCursor(19, 3);
  lcd.write(2);
}

void loop()
{
  //buttons ->
  up.tick();
  down.tick();

  current_id = 1;

  if (up.press())
  {
    target_temps_arr[current_id] += 0.1;
    if (target_temps_arr[current_id] > 100)
    {
      target_temps_arr[current_id] = 100;
    }
  }
  if (up.step())
  {
    target_temps_arr[current_id] += 0.1;
    if (target_temps_arr[current_id] > 100)
    {
      target_temps_arr[current_id] = 100;
    }
  }

  if (down.press())
  {
    target_temps_arr[current_id] -= 0.1;
    if (target_temps_arr[current_id] < 0)
    {
      target_temps_arr[current_id] = 0;
    }
  }
  if (down.step())
  {
    target_temps_arr[current_id] -= 0.1;
    if (target_temps_arr[current_id] < 0)
    {
      target_temps_arr[current_id] = 0;
    }
  }

  if (down.busy() || up.busy())
  {
    force_update = true;
    last_click_timer = millis();
    clicked = true;

    lcd.setCursor(19, 3);
    lcd.print(" ");
  }
  //<- buttons

  if (clicked && millis() - last_click_timer >= 5000)
  {
    clicked = false;
    last_click_timer = millis();
    force_update = true;

    EEPROM.put(5, target_temps_arr[current_id]);

    lcd.setCursor(19, 3);
    lcd.write(2);
  }

  if ((millis() - sensor_read_timer >= 1000) || force_update)
  {
    force_update = false;
    sensor_read_timer = millis();
    //--
    for (byte id = 0; id < 2; id++)
    {
      if (sensors.readTemp(id))
      {
        fatal = false;
        if (temps_prev_arr[id] != temps_arr[id])
        {
          temps_prev_arr[id] = temps_arr[id];
        }
        temps_arr[id] = sensors.getTemp(id);

        if ((float)temps_arr[id] - temps_prev_arr[id] >= 0.1)
        {
          arrow_temp[id] = true;
        }
        if ((float)temps_arr[id] - temps_prev_arr[id] <= 0.1)
        {
          arrow_temp[id] = false;
        }
        //Serial.println(temps_arr[1]);
      }
      else
      {
        if (millis() - startup_timer >= 2000)
        {
          //Serial.println("error");
          fatal = true;

          for (byte beeps_am = 1; beeps_am <= 5; beeps_am++)
          {
            digitalWrite(8, 1);
            delay(100);
            digitalWrite(8, 0);
            delay(100);
          }
        }
        //asm volatile("JMP 0x00");
      }
    }
    sensors.requestTempAll();
    //--
    lcd.home();
    lcd.print("#1 ");
    lcd.print("       ");
    lcd.setCursor(3, 0);
    lcd.print(state ? "Heating" : "Idle");
    lcd.setCursor(0, 1);
    lcd.print("t");
    lcd.write((char)223);
    lcd.print("=");
    lcd.print("       ");
    lcd.setCursor(3, 1);
    if (!fatal)
    {
      lcd.print(temps_arr[0]);
    }
    else
    {
      lcd.print("ERROR");
    }
    lcd.write(arrow_temp[0] ? 1 : 0);
    lcd.print("  ");

    lcd.print("t");
    lcd.write((char)223);
    lcd.print("=");
    if (!fatal)
    {
      lcd.print(temps_arr[1]);
    }
    else
    {
      lcd.print("ERROR");
    }
    lcd.write(arrow_temp[1] ? 1 : 0);

    lcd.setCursor(0, 2);
    lcd.print("t");
    lcd.write((char)223);
    lcd.print("(target)=");
    lcd.print(target_temps_arr[1]);
  }

  //relays ->
  if ((float)target_temps_arr[current_id] - temps_arr[0] >= HYSTERESYS && !fatal && !state)
  {
    digitalWrite(4, 0);
    state = true;

    Timer1.restart();
    time_spent_s = 0;
    time_spent_m = 0;
    time_spent_h = 0;
  }
  if ((target_temps_arr[current_id] <= temps_arr[0] && state) || fatal)
  {
    digitalWrite(4, 1);
    state = false;

    Timer1.restart();
    time_spent_s = 0;
    time_spent_m = 0;
    time_spent_h = 0;
  }
  //<-relays

  if (millis() - couter_timer >= 1000)
  {
    couter_timer = millis();

    lcd.setCursor(11, 0);
    lcd.print("        ");
    lcd.setCursor(11, 0);

    lcd.print(time_spent_h);
    lcd.print(':');
    lcd.print(time_spent_m);
    lcd.print(':');
    lcd.print(time_spent_s);
  }
}

ISR(TIMER1_A)
{
  time_spent_s++;
  if (time_spent_s >= 60)
  {
    time_spent_s = 0;
    time_spent_m++;
    if (time_spent_m >= 60)
    {
      time_spent_m = 0;
      time_spent_h++;
    }
  }
}

void up_tick(void) { up.pressISR(); }
void down_tick(void) { down.pressISR(); }
