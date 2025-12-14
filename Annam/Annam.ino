#include "lcd_4inch.h"
#include "lcd_touch.h"
#include "lcd_gui.h"
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

// Servo motor
#define SERVO_PIN 6
int freq = 20000; // 20 milliseconds(50Hz)
int minPulse = 500; // 500 microseconds (서보 모터의 최소 펄스)
int maxPulse = 2500; // 2500 microseconds (서보 모터의 최대 펄스)

// 엔코더 및 비밀번호 관련 변수 설정
#define outputA A1
#define outputB A0
#define sw A2
#define RST A3

// RFID 설정
#define PN532_SCK (2)
#define PN532_MOSI (3)
#define PN532_SS (4)
#define PN532_MISO (5)
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// 엔코더 동작 [변수]
int value = 0;
int counter = 0;
int aState;
int aLastState;
String num = "";

// 비밀번호 동작 [변수]
int setPassword[4] = {1, 2, 3, 4};
int inputPassword[4] = {0};
int index = 0;
bool isSame = true; // 비밀번호 비교 변수

bool lastButtonState = LOW; // 비밀번호 입력 확인 변수

// 비밀번호 리셋 동작
unsigned long RstPressedTime = 0;
unsigned long RstReleasedTime = 0;
bool isRstBeingPressed = false;

// RFID 동작 [변수]
unsigned long lastRFIDReadTime = 0;
const unsigned long RFIDReadInterval = 1000; // 1초 간격
const unsigned long RFIDReadDuration = 100; // 0.1초 동안 RFID 읽기

// 비교할 UID 설정 (예시로 4바이트 UID를 사용)
uint8_t validUID[4] = {0x9C, 0xC4, 0x77, 0xC1};

void setup() {
    // 5pin Encoder 설정
    pinMode(outputA, INPUT);
    pinMode(outputB, INPUT);
    pinMode(sw, INPUT);

    // Clear Password Button 설정
    pinMode(RST, INPUT);

    // LCD 및 터치스크린 초기화
    TP_Init();
    Lcd_Gpio_Init();

    // 시리얼 및 SPI 초기화
    Serial.begin(115200);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.begin();

    Serial.println("4Inch LCD Init...");
    Lcd_Init();

    // 서보 모터 초기 위치 설정
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);

    // LCD 초기 그래픽
    Gui_draw_str(80, 160, "Enter Password:", &Font24, BLACK, WHITE);
    delay(2000);
    LCD_Clear(BACKGROUND_COLOR);
    Serial.println("Ready...");

    // NFC 초기화
    nfc.begin();
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
        Serial.print("Didn't find PN53x board");
        while (1);
    }
    nfc.SAMConfig();
    Serial.println("Waiting for an NFC card ...");

    // 서보 모터 위치 초기화
    unsigned long startTime = millis();
    while (millis() - startTime < 500) { // 90도
        setServo(90);
    }
}

// 조그셔틀 동작 함수
void handleEncoder() {
    aState = digitalRead(outputA);
    if (aState != aLastState) {
      // 좌측 : (-), 우측 : (+)
        if (digitalRead(outputB) != aState) {
            counter++;
        } else {
            counter--;
        }

        // Counter가 0 아래로 갈 때 9로 순환 (롤오버 되도록 설계)
        if (counter < 0) {
            counter = 19;  // 19를 사용하여 9가 되도록 함 (9 * 2 + 1)
        }
        // 로그 출력을 위한 print
        Serial.print("Position: ");
        Serial.println(counter);

        // 조그셔틀 동작에 의한 value 값 설정.
        value = (counter / 2) % 10;
        num = String(value);

        // 만약 홀수일 경우 (조그셔틀 동작으로 인해 다음과 같이 설계하였음.)
        if (counter % 2 == 1) {
            Gui_fill_color(200, 100, 500, 200, WHITE);
            Gui_draw_str(230, 150, num.c_str(), &Font24, BLACK, WHITE);
            Serial.print("Position: ");
            Serial.println(counter);
        }

        // 딜레이를 통해 버퍼역할 추가(조그셔틀 제어 시 속도조절 목적)
        while (counter % 2 == 0) {
            delay(10);
            break;
        }

        // 조그셔틀 동작확인을 위한 상태 업데이트
        aLastState = aState;
    }
}

// 비밀번호 입력 함수
void handlePasswordInput(bool currentButtonState) {
  // 스위치 처럼 동작시키기 위해 if문 설계(한번 누르면 입력)
    if (currentButtonState != lastButtonState) {
        if (currentButtonState == LOW) {
          // index에 숫자 저장.
            inputPassword[index] = num.toInt();
            
          // 로그 출력을 위한 print
            Serial.print("Index: ");
            Serial.println(index);
            Serial.print("Value: ");
            Serial.println(inputPassword[index]);

          // 비밀번호 출력 함수
            displayPasswordInput(index);
            index++;
        }

        // 비밀번호 입력이 완료되었을 경우(비밀번호는 4자리)
        if (index == 4) {
          // checkPassword 함수를 통해서 비밀번호 비교
            checkPassword();

          // 비교 후 인덱스 초기화(비밀번호 재입력)
            index = 0; // Reset the index
        }

        // 딜레이를 통해서 버튼 버퍼 구현
        delay(10);
    }

    // 버튼(조그셔틀) 동작확인을 위한 상태 업데이트
    lastButtonState = currentButtonState;
}

// 비밀번호 출력 함수(UI)
void displayPasswordInput(int index) {
  // 인덱스만큼 화면에 그리기 (사용자가 어디까지 입력했는지 확인 가능)
    for (int i = 0; i <= index; i++) {
        Gui_draw_line(70 + 100 * i, 75, 100 + 100 * i, 75, BLACK, 1, SOLID);
        Gui_draw_str(75 + 100 * i, 50, "*", &Font24, BLACK, WHITE);
    }
}

// 비밀번호 비교 함수
void checkPassword() {
    isSame = true;
    // 인덱스를 체크하여 비밀번호 비교
    for (int i = 0; i < 4; i++) {
        if (setPassword[i] != inputPassword[i]) {
            isSame = false;
            break;
        }
    }

    if (isSame) {
      // 비밀번호가 맞았을 경우
        handleCorrectPassword();
    } else {
      // 비밀번호가 틀렸을 경우
        handleIncorrectPassword();
    }
}

// 서보모터 제어를 위한 함수
// Servo.h를 사용할 경우 LCD 제어가 안되는 문제로 따로 함수로 구현.
void setServo(int degree) {
  // high, low TIME 변수 선언(PWM으로 제어)
    int hTime = 0;
    int lTime = 0;

// 각도
    if (degree < 0) degree = 0;
    if (degree > 180) degree = 180;

// TIME
    hTime = (int)(minPulse + ((maxPulse - minPulse) / 180.0 * degree));
    lTime = freq - hTime;

// TIME과 각도만큼 서보모터 제어
    for (int i = 0; i < 30; i++) {
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(lTime);
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(hTime);
        digitalWrite(SERVO_PIN, LOW);
    }
}

// 비밀번호가 맞았을 경우
void handleCorrectPassword() {
    Serial.println("비밀번호 일치");

    // 서보 모터 작동 전에 LCD 업데이트
    LCD_Clear(GREEN);
    Gui_draw_str(230, 150, "True", &Font24, BLACK, WHITE);
    delay(500);  // LCD 업데이트 후 약간의 딜레이 추가

    // 서보 모터 제어
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) { // 0도 위치로 3초 동안 유지
        setServo(0);
    }

    startTime = millis(); // startTime 재설정
    while (millis() - startTime < 500) { // 90도 위치로 0.5초 동안 유지
        setServo(90);
    }

    delay(1000);
    LCD_Clear(BACKGROUND_COLOR);
}

// 비밀번호가 틀렸을 경우
void handleIncorrectPassword() {
    Serial.println("비밀번호 불일치");
    LCD_Clear(RED);
    Gui_draw_str(230, 150, "False", &Font24, BLACK, WHITE);
    delay(1000);
    LCD_Clear(BACKGROUND_COLOR);
}

//======== 비밀번호 리셋 함수 ===========//
void handleResetButton() {
    bool currentRstState = digitalRead(RST);

// 리셋버튼 입력 확인
    if (currentRstState == LOW && !isRstBeingPressed) {
        RstPressedTime = millis();
        isRstBeingPressed = true;
    }

// 리셋 버튼이 3초 이상 눌렸을 경우 리셋모드로 전환
    if (currentRstState == LOW && isRstBeingPressed) {
        if (millis() - RstPressedTime > 3000) { // 3초 이상 눌림
            handleResetMode();
            isRstBeingPressed = false; // Reset 상태 초기화
        }
    } else if (currentRstState == HIGH && isRstBeingPressed) {
        isRstBeingPressed = false;
    }
}

// 비밀번호 리셋 모드
void handleResetMode() {
    Serial.println("Reset Mode");
    LCD_Clear(BACKGROUND_COLOR);
    Gui_draw_str(100, 300, "Reset Mode", &Font24, BLACK, WHITE);
    bool exitResetMode = false;
    index = 0; // 인덱스 초기화

    // 리셋모드 동작
    while (!exitResetMode) {
      // 기존 조그셔틀 동작과 동일
        aState = digitalRead(outputA);

        if (aState != aLastState) {
            if (digitalRead(outputB) != aState) {
                counter++;
            } else {
                counter--;
            }

            // Counter가 0 아래로 갈 때 9로 순환
            if (counter < 0) {
                counter = 19;  // 19를 사용하여 9가 되도록 함 (9 * 2 + 1)
            }

            Serial.print("Position: ");
            Serial.println(counter);

            value = (counter / 2) % 10;
            num = String(value);
            if (counter % 2 == 1) {
                Gui_fill_color(200, 100, 500, 200, WHITE);
                Gui_draw_str(230, 150, num.c_str(), &Font24, BLACK, WHITE);
                Serial.print("Position: ");
                Serial.println(counter);
            }

            while (counter % 2 == 0) {
                delay(10);
                break;
            }
            aLastState = aState;
        }
        
        // 비밀번호 입력부, 기존에는 inputPassword[]에 비밀번호를 입력
        // 리셋모드에서는 setPassword[]에 비밀번호를 입력함
        bool currentButtonState = digitalRead(sw);
        if (currentButtonState != lastButtonState) {
            if (currentButtonState == LOW) {
                setPassword[index] = num.toInt();
                Serial.print("index: ");
                Serial.println(index);
                Serial.print("value: ");
                Serial.println(setPassword[index]);

                displayPasswordInput(index);
                index++;
            }
            // 비밀번호를 모두 입력했을 경우(비밀번호는 4자리)
            if (index == 4) {
                Serial.println("Password Set Successfully");
                LCD_Clear(GREEN);
                Gui_draw_str(130, 150, "Changed Password", &Font24, BLACK, WHITE);
                delay(1000);
                LCD_Clear(BACKGROUND_COLOR);
                index = 0;

                // 리셋모드 파악 변수를 true로 바꿔 리셋모드 탈출
                exitResetMode = true;
            }
            // 버튼 입력 감지 후 작은 딜레이 추가 (버튼 눌림이 연속적으로 감지되는 것을 방지)
            delay(10);
        }
        // 버튼(조그셔틀) 동작확인을 위한 상태 업데이트
        lastButtonState = currentButtonState;
    }
}

// RFID 비교 함수
bool compareUID(uint8_t *uid, uint8_t uidLength, uint8_t *validUID, uint8_t validUIDLength) {
    if (uidLength != validUIDLength) {
        return false;
    }
    for (int i = 0; i < uidLength; i++) {
        if (uid[i] != validUID[i]) {
            return false;
        }
    }
    return true;
}

// RFID 감지 함수(RFID라이브러리 내의 예제파일 참고)
void handleRFID() {
    static unsigned long lastReadTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastReadTime >= RFIDReadInterval) {
        lastReadTime = currentTime;

        unsigned long readStartTime = millis();
        while (millis() - readStartTime < RFIDReadDuration) {
            uint8_t success;
            uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
            uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

            if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
                Serial.println("Found an NFC card");
                Serial.print("UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
                Serial.print("UID Value: "); nfc.PrintHex(uid, uidLength); Serial.println();

                if (compareUID(uid, uidLength, validUID, sizeof(validUID))) {
                    Serial.println("Valid NFC card detected.");
                    handleCorrectPassword();
                } else {
                    Serial.println("Invalid NFC card.");
                }
                break;
            }
        }
    }
}

// 함수 모듈화를 통해서 코드 간결화
void loop() {
    handleEncoder();
    handlePasswordInput(digitalRead(sw));
    handleResetButton();
    handleRFID();
}
