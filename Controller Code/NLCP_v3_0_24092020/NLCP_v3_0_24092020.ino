
//Include Files
#include <ModbusRtu.h>
//Change MODBUS ID Here
#define ID   9
//Raw ADC Values
int adc_1 = 0, adc_2 = 0;
//Voltage Values
int volt_1 = 0, volt_2 = 0;
//ADC Pins for Supply Monitoring
int Supply_1 = A0, Supply_2 = A1;
//Pins for buzzers, Relays
int Buzz = 3, Sup_relay_1P = 6, Sup_relay_1N = 8, Sup_relay_2P = 7, Sup_relay_2N = 9;
//Pins for enable
int Enbl_1[] = {A6, A10, A14, 24, 27, 30, 34, 40};
int Enbl_2[] = {A8, A12, 22, 26, 29, 32, 36, 38};
//Pins for Diagnose Pins
int Diag_1[] = {A5, A9, A13, 23, 28, 31, 35, 41};
int Diag_2[] = {A7, A11, A15, 25, 39, 33, 37, 20};
//Fault Line
int Fault_line = A3;
//Variable for supply switching
int change = 1;
//MODBUS Data Array
uint16_t MBUS_Data[10];
//MODBUS Config
Modbus slave(ID, 0, 0);
//MODBUS LED state
int8_t state = 0; unsigned long MSCAN;

unsigned long timer_1 = 0, timer_2 = 0, timer_3 = 0, timer_4 = 0, timer_5 = 0;
bool local_data[7][16] = {0};
bool flasher = 0;
unsigned long FlashTime = 0;
unsigned long FlashDelay = 1000;

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(Enbl_1[i], OUTPUT);
    pinMode(Enbl_2[i], OUTPUT);
    pinMode(Diag_1[i], OUTPUT);
    pinMode(Diag_2[i], OUTPUT);
  }
  pinMode(Sup_relay_1P, OUTPUT);
  pinMode(Sup_relay_1N, OUTPUT);
  pinMode(Sup_relay_2P, OUTPUT);
  pinMode(Sup_relay_2N, OUTPUT);
  pinMode(Fault_line, INPUT_PULLUP);
  for (int i = 0; i < 8; i++) {
    digitalWrite( Enbl_1[i], LOW);
    digitalWrite( Enbl_2[i], LOW);
    digitalWrite( Diag_1[i], LOW);
    digitalWrite( Diag_2[i], LOW);
  }
  //Serial.begin(9600);
  slave.begin(115200);
  Supply_Monitoring_Auto();
}



/*MODBUS Registers
  MBUS_Data[0][0] = Lamp A1 - 0
  MBUS_Data[0][1] = Lamp B1
  MBUS_Data[0][2] = Lamp C1
  MBUS_Data[0][3] = Lamp D1
  MBUS_Data[0][4] = Lamp E1
  MBUS_Data[0][5] = Lamp F1
  MBUS_Data[0][6] = Lamp G1
  MBUS_Data[0][7] = Lamp H1
  MBUS_Data[0][8] = Lamp A2
  MBUS_Data[0][9] = Lamp B2
  MBUS_Data[0][10] = Lamp C2
  MBUS_Data[0][11] = Lamp D2
  MBUS_Data[0][12] = Lamp E2
  MBUS_Data[0][13] = Lamp F2
  MBUS_Data[0][14] = Lamp G2
  MBUS_Data[0][15] = Lamp H2 - 15

  MBUS_Data[1][0] = Acknowledge A1 - 16
  MBUS_Data[1][1] = Acknowledge B1
  MBUS_Data[1][2] = Acknowledge C1
  MBUS_Data[1][3] = Acknowledge D1
  MBUS_Data[1][4] = Acknowledge E1
  MBUS_Data[1][5] = Acknowledge F1
  MBUS_Data[1][6] = Acknowledge G1
  MBUS_Data[1][7] = Acknowledge H1
  MBUS_Data[1][8] = Acknowledge A2
  MBUS_Data[1][9] = Acknowledge B2
  MBUS_Data[1][10] = Acknowledge C2
  MBUS_Data[1][11] = Acknowledge D2
  MBUS_Data[1][12] = Acknowledge E2
  MBUS_Data[1][13] = Acknowledge F2
  MBUS_Data[1][14] = Acknowledge G2
  MBUS_Data[1][15] = Acknowledge H2 - 31

  MBUS_Data[2][0] = Supply Selection Mode ( 0 = Automatic, 1 = Manual) - 32
  MBUS_Data[2][1] = Selected Supply ( 0 = Supply 1, 1 = Supply 2)
  MBUS_Data[2][2] = Lamp Selection Mode ( 0 = Automatic, 1 = Manual)
  MBUS_Data[2][3] =
  MBUS_Data[2][4] =
  MBUS_Data[2][5] =
  MBUS_Data[2][6] =
  MBUS_Data[2][7] =
  MBUS_Data[2][8] =
  MBUS_Data[2][9] =
  MBUS_Data[2][10] =
  MBUS_Data[2][11] =
  MBUS_Data[2][12] = Selected Supply ( 0 = Supply 1, 1 = Supply 2) ( manual Mode)
  MBUS_Data[2][13] = Supply 1 Warning
  MBUS_Data[2][14] = Supply 2 Warning
  MBUS_Data[2][15] = Buzzer - 47

  MBUS_Data[3] = Power Supply 1 Voltage - 48
  MBUS_Data[4] = Power Supply 2 Voltage - 64

  MBUS_Data[5][0] = Fault A1 - 80
  MBUS_Data[5][1] = Fault B1
  MBUS_Data[5][2] = Fault C1
  MBUS_Data[5][3] = Fault D1
  MBUS_Data[5][4] = Fault E1
  MBUS_Data[5][5] = Fault F1
  MBUS_Data[5][6] = Fault G1
  MBUS_Data[5][7] = Fault H1
  MBUS_Data[5][8] = Fault A2
  MBUS_Data[5][9] = Fault B2
  MBUS_Data[5][10] = Fault C2
  MBUS_Data[5][11] = Fault D2
  MBUS_Data[5][12] = Fault E2
  MBUS_Data[5][13] = Fault F2
  MBUS_Data[5][14] = Fault G2
  MBUS_Data[5][15] = Fault H2 - 95

  MBUS_Data[6][0] = Flash A1 - 96
  MBUS_Data[6][1] = Flash B1
  MBUS_Data[6][2] = Flash C1
  MBUS_Data[6][3] = Flash D1
  MBUS_Data[6][4] = Flash E1
  MBUS_Data[6][5] = Flash F1
  MBUS_Data[6][6] = Flash G1
  MBUS_Data[6][7] = Flash H1
  MBUS_Data[6][8] = Flash A2
  MBUS_Data[6][9] = Flash B2
  MBUS_Data[6][10] = Flash C2
  MBUS_Data[6][11] = Flash D2
  MBUS_Data[6][12] = Flash E2
  MBUS_Data[6][13] = Flash F2
  MBUS_Data[6][14] = Flash G2
  MBUS_Data[6][15] = Flash H2 - 111

*/


void Supply_Watchdog() {
  adc_1 = analogRead(Supply_1);
  adc_2 = analogRead(Supply_2);
  volt_1 = map(adc_1, 0, 1023, 0, 55);
  volt_2 = map(adc_2, 0, 1023, 0, 55);
  if (volt_1 < 22) {
    local_data[2][13] = 1;
  }
  else {
    local_data[2][13] = 0;
  }
  if (volt_2 < 22) {
    local_data[2][14] = 1;
  }
  else {
    local_data[2][14] = 0;
  }
}

void Supply_Monitoring_Auto() {
  if (volt_1 >= volt_2) {
    digitalWrite(Sup_relay_1P, HIGH);
    digitalWrite(Sup_relay_1N, HIGH);
    delay(100);
    digitalWrite(Sup_relay_2P, LOW);
    digitalWrite(Sup_relay_2N, LOW);
    local_data[2][12] = 0;
  }
  if (volt_2 > volt_1) {
    digitalWrite(Sup_relay_1P, LOW);
    digitalWrite(Sup_relay_1N, LOW);
    delay(100);
    digitalWrite(Sup_relay_2P, HIGH);
    digitalWrite(Sup_relay_2N, HIGH);
    local_data[2][12] = 1;
  }
}

void Supply_Monitoring_Manual() {
  switch (local_data[2][1]) {
    case 0:
      digitalWrite(Sup_relay_1P, HIGH);
      digitalWrite(Sup_relay_1N, HIGH);
      delay(100);
      digitalWrite(Sup_relay_2P, LOW);
      digitalWrite(Sup_relay_2N, LOW);
      local_data[2][12] = 0;
      break;
    case 1:
      digitalWrite(Sup_relay_1P, LOW);
      digitalWrite(Sup_relay_1N, LOW);
      delay(100);
      digitalWrite(Sup_relay_2P, HIGH);
      digitalWrite(Sup_relay_2N, HIGH);
      local_data[2][12] = 1;
      break;
  }
}

void lamp_IO() {
  for ( int i = 0; i < 8; i++) {
    digitalWrite(Enbl_1[i], local_data[0][i]);
    digitalWrite(Enbl_2[i], local_data[0][i + 8]);
  }
}

void lamp_IO_auto() {
  for ( int i = 0; i < 8; i++) {
    if (local_data[5][i]) {
      digitalWrite(Enbl_2[i], local_data[0][i + 8]);
    }
    else {
      digitalWrite(Enbl_1[i], local_data[0][i]);
      digitalWrite(Enbl_2[i], local_data[0][i + 8]);
    }
  }
}

void lamp_Fault_Monitoring() {
  for ( int i = 0; i < 8; i++) {
    if (local_data[0][i]) {
      digitalWrite(Diag_1[i], HIGH);
      delay(20);
      local_data[5][i] = !digitalRead(Fault_line);
      digitalWrite(Diag_1[i], LOW);
    }
    if (local_data[0][i + 8]) {
      digitalWrite(Diag_2[i], HIGH);
      delay(20);
      local_data[5][i + 8] = !digitalRead(Fault_line);
      digitalWrite(Diag_2[i], LOW);
    }
  }
}

void lamp_auto() {
  for ( int i = 0; i < 8; i++) {
    if (local_data[0][i]) {
      if (local_data[5][i]) {
        local_data[0][i + 8] = local_data[0][i] ;
      }
      else {
        local_data[0][i + 8] = !local_data[0][i] ;
      }
    }
    if (!local_data[0][i]) {
      local_data[0][i + 8] = local_data[0][i] ;
    }
  }
}

void flashing() {
  for (int i = 0; i < 8; i++) {
    local_data[6][i] = (local_data[0][i]) * (local_data[5][i]) * (!local_data[1][i]);
    local_data[6][i + 8] = (local_data[0][i + 8]) * (local_data[5][i + 8]) * (!local_data[1][i + 8]);
    if (local_data[6][i]) {
      digitalWrite(Buzz, flasher);

    }
    else if (local_data[6][i + 8]) {
      digitalWrite(Buzz, flasher);
    }
    else {
      digitalWrite(Buzz, LOW);
    }
  }
}






void loop() {
  state = slave.poll( MBUS_Data, 8 );
  if (state > 4) {
    MSCAN = millis() + 100;
    digitalWrite(13, HIGH);
  }
  if (millis() > MSCAN) {
    digitalWrite(13, LOW );
  }


  if (millis() - FlashTime >= FlashDelay) {
    flasher = !flasher;
    FlashTime = millis();
  }


  for (int i = 0; i < 8; i++) {
    local_data[0][i] = bitRead(MBUS_Data[0], i);
  }
  if (bitRead(MBUS_Data[2], 2)) {
    for (int i = 8; i < 16; i++) {
      local_data[0][i] = bitRead(MBUS_Data[0], i);
    }
  }
  else {
    for (int i = 8; i < 16; i++) {
      bitWrite(MBUS_Data[0], i, local_data[0][i]);
    }
  }

  for (int i = 0; i < 16; i++) {
    local_data[1][i] = bitRead(MBUS_Data[1], i);
  }

  for (int j = 5; j < 7; j++) {
    for (int i = 0; i < 16; i++) {
      bitWrite(MBUS_Data[j], i, local_data[j][i]);
    }
  }
  state = slave.poll( MBUS_Data, 8 );
  if (state > 4) {
    MSCAN = millis() + 100;
    digitalWrite(13, HIGH);
  }
  if (millis() > MSCAN) {
    digitalWrite(13, LOW );
  }
  MBUS_Data[3] = volt_1;
  MBUS_Data[4] = volt_2;
  //Supply





  if (millis() - timer_1 >= 250) {
    state = slave.poll( MBUS_Data, 8 );
    if (state > 4) {
      MSCAN = millis() + 100;
      digitalWrite(13, HIGH);
    }
    if (millis() > MSCAN) {
      digitalWrite(13, LOW );
    }


    bitWrite(MBUS_Data[2], 13, local_data[2][13]);
    bitWrite(MBUS_Data[2], 14, local_data[2][14]);

    Supply_Watchdog();
    if (bitRead(MBUS_Data[2], 0)) {
      local_data[2][1] = bitRead(MBUS_Data[2], 1);
      Supply_Monitoring_Manual();
      bitWrite(MBUS_Data[2], 12, local_data[2][12]);
    }
    else {
      Supply_Monitoring_Auto();
      bitWrite(MBUS_Data[2], 12, local_data[2][12]);
    }
    state = slave.poll( MBUS_Data, 8 );
    if (state > 4) {
      MSCAN = millis() + 100;
      digitalWrite(13, HIGH);
    }
    if (millis() > MSCAN) {
      digitalWrite(13, LOW );
    }
    timer_1 = millis();
  }

  //Lamp
  if (millis() - timer_2 >= 350) {
    state = slave.poll( MBUS_Data, 8 );
    if (state > 4) {
      MSCAN = millis() + 100;
      digitalWrite(13, HIGH);
    }
    if (millis() > MSCAN) {
      digitalWrite(13, LOW );
    }
    if (bitRead(MBUS_Data[2], 6)) {
      lamp_IO();
      lamp_Fault_Monitoring();
    }
    else {

      lamp_IO_auto();
      lamp_Fault_Monitoring();
      lamp_auto();

    }
    flashing();
    state = slave.poll( MBUS_Data, 8 );
    if (state > 4) {
      MSCAN = millis() + 100;
      digitalWrite(13, HIGH);
    }
    if (millis() > MSCAN) {
      digitalWrite(13, LOW );
    }
    timer_2 = millis();

  }





}
