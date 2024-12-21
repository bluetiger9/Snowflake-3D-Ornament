/** Imports **/
#include <cstdarg>
#include "nrf.h"
#include <BLEPeripheral.h>
#include "BLESerial.h"

/** HARDWARE and PINOUT **/

/** RGB LED Pins **/
class LedPins {
  public:
    const uint8_t red;
    const uint8_t green;
    const uint8_t blue;

    LedPins(uint8_t red, uint8_t green, uint8_t blue)
      : red(red), green(green), blue(blue) { }
};

/** LED 1 pins **/
static const uint8_t PIN0_29 = 29;
static const uint8_t PIN0_30 = 30;
static const uint8_t PIN0_31 = 31;

static const LedPins LED_1_PINS(PIN0_30, PIN0_29, PIN0_31);

/** LED 2  pins*/
static const uint8_t PIN0_25 = 25;
static const uint8_t PIN0_26 = 26;
static const uint8_t PIN0_27 = 27;

static const LedPins LED_2_PINS(PIN0_27, PIN0_26, PIN0_25);

/** LED 3 pins **/
static const uint8_t PIN0_08 = 8;
static const uint8_t PIN0_09 = 9;
static const uint8_t PIN0_10 = 10;

static const LedPins LED_3_PINS(PIN0_10, PIN0_09, PIN0_08);

void initPins() {
  // Set pin 9 and 10 as GPIO (disable NFC)
  NRF_NFCT->TASKS_DISABLE = 1;                      // Disable NFC
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;           // Enable write to UICR
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy); // Wait for ready
  NRF_UICR->NFCPINS = 0;                            // Set pins 9 and 10 to GPIO
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;           // Disable write to UICR
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy); // Wait for ready

  // initialize LED pins as output (HIGH = off)
  enablePins();

  // start blink
  digitalWrite(LED_1_PINS.red, LOW);
  digitalWrite(LED_2_PINS.red, LOW);
  digitalWrite(LED_3_PINS.red, LOW);
  delay(100);
  digitalWrite(LED_1_PINS.red, LOW);
  digitalWrite(LED_2_PINS.red, LOW);
  digitalWrite(LED_3_PINS.red, LOW);
}

void enablePins() {
  pinMode(LED_1_PINS.red, OUTPUT);
  digitalWrite(LED_1_PINS.red, HIGH);

  pinMode(LED_1_PINS.green, OUTPUT);
  digitalWrite(LED_1_PINS.green, HIGH);

  pinMode(LED_1_PINS.blue, OUTPUT);
  digitalWrite(LED_1_PINS.blue, HIGH);

  pinMode(LED_2_PINS.red, OUTPUT);
  digitalWrite( LED_2_PINS.red, HIGH);

  pinMode(LED_2_PINS.green, OUTPUT);
  digitalWrite(LED_2_PINS.green, HIGH);

  pinMode(LED_2_PINS.green, OUTPUT);
  digitalWrite(LED_2_PINS.green, HIGH);

  pinMode(LED_3_PINS.red, OUTPUT);
  digitalWrite(LED_3_PINS.red, HIGH);

  pinMode(LED_3_PINS.green, OUTPUT);
  digitalWrite(LED_3_PINS.green, HIGH);

  pinMode(LED_3_PINS.blue, OUTPUT);
  digitalWrite(LED_3_PINS.blue, HIGH);
}

void disablePins() {
  digitalWrite(LED_1_PINS.red, HIGH);
  digitalWrite(LED_1_PINS.green, HIGH);
  digitalWrite(LED_1_PINS.blue, HIGH);
  digitalWrite(LED_2_PINS.red, HIGH);
  digitalWrite(LED_2_PINS.green, HIGH);
  digitalWrite(LED_2_PINS.blue, HIGH);
  digitalWrite(LED_3_PINS.red, HIGH);
  digitalWrite(LED_3_PINS.green, HIGH);
  digitalWrite(LED_3_PINS.blue, HIGH);
  pinMode(LED_1_PINS.red, INPUT);
  pinMode(LED_1_PINS.green, INPUT);
  pinMode(LED_1_PINS.blue, INPUT);
  pinMode(LED_2_PINS.red, INPUT);
  pinMode(LED_2_PINS.green, INPUT);
  pinMode(LED_2_PINS.green, INPUT);
  pinMode(LED_3_PINS.red, INPUT);
  pinMode(LED_3_PINS.green, INPUT);
  pinMode(LED_3_PINS.blue, INPUT);
}

void initDCDC() {
  // enable DC/DC converter to reduce power
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE); // use softdevice
  // NRF_POWER->DCDCEN = 1; // for cases where no softdevice is used
}

/** PWM **/

/** The NRF52x's hardware PWM instances **/
static NRF_PWM_Type* PWMS[3] = { NRF_PWM0, NRF_PWM1, NRF_PWM2 };

/** Static buffer for the current PWM values (3 PWM instances x 4 channels) **/
static uint16_t pwmValues[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/** Maximum PWM intensity value (12 bit) **/
const uint16_t IMAX = 1024 - 1;

/** Initialize a PWM channel with up to 4 hardware pins (outputs). */
void pwmInit(uint8_t pwm_inst, uint8_t pin0, uint8_t pin1, uint8_t pin2, uint8_t pin3) {
  // Map pins
  uint32_t ulPin0 = pin0 < 64 ? g_ADigitalPinMap[pin0] : 0xFF;
  uint32_t ulPin1 = pin1 < 64 ? g_ADigitalPinMap[pin1] : 0xFF;
  uint32_t ulPin2 = pin2 < 64 ? g_ADigitalPinMap[pin2] : 0xFF;
  uint32_t ulPin3 = pin3 < 64 ? g_ADigitalPinMap[pin3] : 0xFF;

  // Init PWM values to 0 for all four channels
  uint8_t seqIdx = 4 * pwm_inst;
  pwmValues[seqIdx] = 0;
  pwmValues[seqIdx + 1] = 0;
  pwmValues[seqIdx + 2] = 0;
  pwmValues[seqIdx + 3] = 0;

  // Get the requested PWM instance
  NRF_PWM_Type* pwm = PWMS[pwm_inst];

  // Enable and configure PWM output pins (if needed)
  pwm->PSEL.OUT[0] = ulPin0 != 0xFF ? ((ulPin0 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos)) : (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
  pwm->PSEL.OUT[1] = ulPin1 != 0xFF ? ((ulPin1 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos)) : (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
  pwm->PSEL.OUT[2] = ulPin2 != 0xFF ? ((ulPin2 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos)) : (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
  pwm->PSEL.OUT[3] = ulPin3 != 0xFF ? ((ulPin3 << PWM_PSEL_OUT_PIN_Pos) | (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos)) : (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);

  // Enable the PWM instance
  pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  pwm->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos); // count up
  pwm->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos); // no pre-scaler
  pwm->COUNTERTOP = IMAX << PWM_COUNTERTOP_COUNTERTOP_Pos; // 12 bit
  pwm->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos); // no-loop   // (down) every channel has its own value in the buffer
  pwm->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  pwm->SEQ[0].PTR = ((uint32_t)(&pwmValues[seqIdx]) << PWM_SEQ_PTR_PTR_Pos);  // pointer to the right section from the value buffer
  pwm->SEQ[0].CNT = 4;  // 4 values in the buffer
  pwm->SEQ[0].REFRESH = 0;  // no refresh
  pwm->SEQ[0].ENDDELAY = 0;     // no delay at the end
  pwm->TASKS_SEQSTART[0] = 1;   // request start of the sequence
}

void pwmDisable(uint8_t pwm_inst) { // does not works
  NRF_PWM_Type* pwm = PWMS[pwm_inst];
  pwm->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);  
  pwm->TASKS_STOP = 1;  
  // Response to STOP task, emitted when PWM pulses are no longer generated
  while (pwm->EVENTS_STOPPED == 0) ;
}

void pwmEnable(uint8_t pwm_inst) {
  NRF_PWM_Type* pwm = PWMS[pwm_inst];
  pwm->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
}

void pwmWrite(uint8_t pwm_inst, uint8_t pwm_chan, uint16_t value) {
  // Select the appropiate buffer index for the requested PWM instance and channel
  const uint8_t seqIdx = 4 * pwm_inst + pwm_chan;

  if (value == pwmValues[seqIdx]) {
    // Same as old value
    return;
  }

  // Update the appropiate value in the PWM value buffer
  pwmValues[seqIdx] = value;

  // Request sequence start
  NRF_PWM_Type* pwm = PWMS[pwm_inst];
  pwm->TASKS_SEQSTART[0] = 0x1UL;
}

/** PWM Channel **/
class PWMChan {
  public:
    /** The hardware PWM instance to use */
    const uint8_t inst;

    /** The PWM channel to use from the requested hardware PWM instance*/
    const uint8_t chan;

    PWMChan(uint8_t pwmInst, uint8_t pwmChan)
      : inst(pwmInst), chan(pwmChan) { }

    void set(const uint16_t value) {
      pwmWrite(this->inst, this->chan, value);
    }
};

void initPWM() {
  // asign the RGB LEDs to PWM channels 0-2
  pwmInit(0, LED_1_PINS.red, LED_1_PINS.green, LED_1_PINS.blue, 0xFF);
  pwmInit(1, LED_2_PINS.red, LED_2_PINS.green, LED_2_PINS.blue, 0xFF);
  pwmInit(2, LED_3_PINS.red, LED_3_PINS.green, LED_3_PINS.blue, 0xFF);
}

void disablePWM() {
  initPWM();
  pwmDisable(0);
  pwmDisable(1);
  pwmDisable(2);
}

void enablePWM() {
  initPWM();
  // pwmEnable(0);
  // pwmEnable(1);
  // pwmEnable(2);
}

/** COLOR, PALETTES and PATTERNS */

/** RGB Color */
class Color {
  public:
    uint16_t red;
    uint16_t green;
    uint16_t blue;

    Color() : red(128), green(128), blue(128) { }

    Color(uint16_t red, uint16_t green, uint16_t blue)
      : red(red), green(green), blue(blue) { }

    Color(const Color &obj)
      : red(obj.red), green(obj.green), blue(obj.blue) { }

    Color fade(float perc) {
      return Color(perc * red, perc * green, perc * blue);
    }

    static Color create(float red, float green, float blue) {
      return Color(IMAX * red, IMAX * green, IMAX * blue);
    }
};

/** Color Palette with a maximum of 32 colors */
class ColorPal {
  public:
    uint8_t size = 0;
    Color colors[32];

    ColorPal() {}

    ColorPal& add(Color color) {
      if (size <= 31) {;
        colors[size] = color;
        size++;
      }
      return *this;
    }

    void reset() {
      this->size = 0;
    }
};

/** Fade Modes */
const uint8_t FADE_OFF = 0;
const uint8_t FADE_ON = 1;
const uint8_t FADE_ON_FAST_START = 2;
const uint8_t FADE_ON_FAST_END = 3;

/** A light pattern. Can be generic (with color pattern applied over it later) or specific (with pre-defined colors). */
class Pattern {
  public:
    Color *colors;
    uint8_t *fades;
    uint32_t *delaysMs;
    uint16_t cap;
    uint16_t size;
    uint32_t totalTime;

    Pattern(uint16_t nrColors) {
      this->colors = new Color[nrColors];
      this->fades = new uint8_t[nrColors];
      this->delaysMs = new uint32_t[nrColors];
      this->cap = nrColors;
      this->size = 0;
      this->totalTime = 0;
    }

    void reset() {
      this->size = 0;
      this->totalTime = 0;
    }

    Pattern& addColor(Color color, uint8_t fade, uint32_t delayMs) {
      if (this->size >= this->cap) {
        return *this;
      }

      this->colors[this->size] = color;
      this->fades[this->size] = fade;
      this->delaysMs[this->size] = delayMs;
      this->totalTime += delayMs;
      this->size++;
      return *this;
    }

    /**
     * Apply a color palette over this pattern.
     *
     * Positions with the same R, G and B value are considered generic,
     * and palette colors will be applied over them. Other colors from
     * the pattern will remain unchanged.
     *
     * The pattern will repeat for each color from the palette.
     */
    void apply(Pattern *pattern, ColorPal &colorPal) {
      reset();
      for (uint16_t cnr = 0; cnr < colorPal.size; ++cnr) {
        Color palCol = colorPal.colors[cnr];
        for (uint16_t nr = 0; nr < pattern->size; ++nr) {
          Color patCol = pattern->colors[nr];
          if (patCol.red == patCol.green && patCol.green == patCol.blue) {
            Color palColFaded = palCol.fade((float) patCol.red / IMAX);
            this->addColor(palColFaded, pattern->fades[nr], pattern->delaysMs[nr]);
          } else {
            this->addColor(patCol, pattern->fades[nr], pattern->delaysMs[nr]);
          }
        }
      }
    }
};

/** The size Colors buffer **/
const uint16_t NR_COLORS = 1024;

/** The Colors buffer **/
Color colors[NR_COLORS];

/** Initializes predefined colors **/
void initColors() {
  colors[0] = Color::create(0.0f, 0.0f, 0.0f);   // Black
  colors[1] = Color::create(1.0f, 0.0f, 0.0f);   // Red
  colors[2] = Color::create(0.0f, 1.0f, 0.0f);   // Green
  colors[3] = Color::create(1.0f, 1.0f, 0.0f);   // Yellow
  colors[4] = Color::create(0.0f, 0.0f, 1.0f);   // Blue
  colors[5] = Color::create(1.0f, 0.0f, 1.0f);   // Magenta
  colors[6] = Color::create(0.0f, 1.0f, 1.0f);   // Cyan
  colors[7] = Color::create(1.0f, 1.0f, 1.0f);   // White
  colors[8] = Color::create(0.5f, 0.5f, 0.5f);   // Bright Black (Gray)
  colors[9] = Color::create(1.0f, 0.5f, 0.5f);   // Bright Red
  colors[10] = Color::create(0.5f, 1.0f, 0.5f);  // Bright Green
  colors[11] = Color::create(1.0f, 1.0f, 0.5f);  // Bright Yellow
  colors[12] = Color::create(0.5f, 0.5f, 1.0f);  // Bright Blue
  colors[13] = Color::create(1.0f, 0.5f, 1.0f);  // Bright Magenta
  colors[14] = Color::create(0.5f, 1.0f, 1.0f);  // Bright Cyan
  colors[15] = Color::create(1.0f, 1.0f, 1.0f);  // Bright White

  colors[16] = Color::create(1.0f, 0.5f, 0.0f);  // Orange
  colors[17] = Color::create(1.0f, 0.0f, 0.5f);  // Pinkish
  colors[18] = Color::create(0.5f, 1.0f, 0.0f);  // Yellowish Green
  colors[19] = Color::create(0.0f, 1.0f, 0.5f);  // Turquoise
  colors[20] = Color::create(0.5f, 0.0f, 1.0f);  // Purple
  colors[21] = Color::create(0.0f, 0.5f, 1.0f);  // Greenish Blue

  colors[22] = Color::create(1.0f, 0.74f, 0.0f);  // Orange
  colors[23] = Color::create(1.0f, 0.0f, 0.74f);  // Pinkish
  colors[24] = Color::create(0.74f, 1.0f, 0.0f);  // Yellowish Green
  colors[25] = Color::create(0.0f, 1.0f, 0.74f);  // Turquoise
  colors[26] = Color::create(0.74f, 0.0f, 1.0f);  // Purple
  colors[27] = Color::create(0.0f, 0.74f, 1.0f);  // Greenish Blue
}

/** The size of the Color Palettes buffer **/
const uint16_t NR_COLPALS = 64;

/** The position used for the dynamically applied Color Palettes **/
const uint16_t COLPAL_BUF = NR_COLPALS - 1;

/** The Color Palettes buffer **/
ColorPal colorPals[NR_COLPALS];

/** Initializes predefined colors palettes **/
void initColorPals() {
  // RGB
  colorPals[0].add(colors[1]).add(colors[2]).add(colors[4]);
  colorPals[1].add(colors[1]);  // R
  colorPals[2].add(colors[2]);  // G
  colorPals[3].add(colors[4]);  // B

  colorPals[4].add(colors[1]).add(colors[2]); // R + G
  colorPals[5].add(colors[1]).add(colors[4]); // R + B
  colorPals[6].add(colors[2]).add(colors[4]); // G + B

  colorPals[7].add(colors[1]).add(colors[3]); // R + Yellow
  colorPals[8].add(colors[1]).add(colors[5]); // R + Magenta
  colorPals[9].add(colors[2]).add(colors[3]); // G + Yellow
  colorPals[10].add(colors[2]).add(colors[6]); // G + Cyan
  colorPals[11].add(colors[4]).add(colors[5]); // B + Magenta
  colorPals[12].add(colors[4]).add(colors[6]); // B + Cyan

  colorPals[13].add(colors[1]).add(colors[3]).add(colors[5]); // R + Yellow + Magenta
  colorPals[14].add(colors[2]).add(colors[3]).add(colors[6]); // G + Yellow + Cyan
  colorPals[15].add(colors[4]).add(colors[5]).add(colors[6]); // B + Magenta + Cyan

  // A. Classic Christmas Palette
  colorPals[16]
    .add(Color::create(1.0f, 0.0f, 0.0f))    // Holly Red
    .add(Color::create(0.0f, 0.5f, 0.0f))    // Evergreen
    .add(Color::create(1.0f, 1.0f, 1.0f))    // Snow White
    .add(Color::create(1.0f, 0.84f, 0.0f));  // Gold

  // B. Winter Wonderland Palette
  colorPals[17]
    .add(Color::create(0.67f, 0.84f, 1.0f))  // Ice Blue
    .add(Color::create(1.0f, 1.0f, 1.0f))    // Snow White
    .add(Color::create(0.85f, 0.92f, 1.0f))  // Frosty Blue
    .add(Color::create(0.0f, 0.5f, 0.5f));   // Deep Teal

  // C. Candy Cane Palette
  colorPals[18]
    .add(Color::create(0.6f, 0.3f, 0.0f))    // Brown (Tree Trunk)
    .add(Color::create(0.0f, 0.4f, 0.2f))    // Pine Green
    .add(Color::create(0.8f, 0.6f, 0.4f))    // Beige (Warm Light)
    .add(Color::create(0.9f, 0.8f, 0.6f));   // Candlelight

  // D. Santa's Sleigh Palette
  colorPals[19]
    .add(Color::create(1.0f, 0.0f, 0.0f))    // Santa Red
    .add(Color::create(1.0f, 0.84f, 0.0f))   // Gold
    .add(Color::create(0.3f, 0.0f, 0.3f))    // Velvet Purple
    .add(Color::create(1.0f, 1.0f, 1.0f));   // Snow White
  
  colorPals[20].add(colors[1]).add(colors[3]).add(colors[7]).add(colors[5]); // R + Yellow + Magenta + White
  colorPals[21].add(colors[2]).add(colors[3]).add(colors[7]).add(colors[6]); // G + Yellow + Cyan + White
  colorPals[22].add(colors[4]).add(colors[5]).add(colors[7]).add(colors[6]); // B + Magenta + Cyan + White

  colorPals[23].add(colors[1]).add(colors[7]).add(colors[2]); // R + W + G
  colorPals[24].add(colors[1]).add(colors[7]).add(colors[4]); // R + W + B
  colorPals[25].add(colors[2]).add(colors[7]).add(colors[4]); // G + W + B

  colorPals[26].add(colors[1]).add(colors[3]).add(colors[7]); // R + Yellow + W
  colorPals[27].add(colors[1]).add(colors[5]).add(colors[7]); // R + Magenta + W
  colorPals[28].add(colors[2]).add(colors[3]).add(colors[7]); // G + Yellow + W
  colorPals[29].add(colors[2]).add(colors[6]).add(colors[7]); // G + Cyan + W
  colorPals[30].add(colors[4]).add(colors[5]).add(colors[7]); // B + Magenta + W
  colorPals[31].add(colors[4]).add(colors[6]).add(colors[7]); // B + Cyan + W

  colorPals[32].add(colors[1]).add(colors[16]).add(colors[17]); // R + Orange + Pinkish
  colorPals[33].add(colors[2]).add(colors[18]).add(colors[19]); // G + Yellowish Green + Turquoise
  colorPals[34].add(colors[4]).add(colors[20]).add(colors[21]); // B + Purple + Greensish Blue

  colorPals[35].add(colors[1]).add(colors[22]).add(colors[23]); // R + Orange + Pinkish
  colorPals[36].add(colors[2]).add(colors[24]).add(colors[25]); // G + Yellowish Green + Turquoise
  colorPals[37].add(colors[4]).add(colors[26]).add(colors[27]); // B + Purple + Greensish Blue

  colorPals[38].add(colors[1]).add(colors[16]).add(colors[7]).add(colors[17]); // R + Orange + Pinkish + White
  colorPals[39].add(colors[2]).add(colors[18]).add(colors[7]).add(colors[19]); // G + Yellowish Green + Turquoise + White
  colorPals[40].add(colors[4]).add(colors[20]).add(colors[7]).add(colors[21]); // B + Purple + Greensish Blue + White

  colorPals[41].add(colors[1]).add(colors[22]).add(colors[7]).add(colors[23]); // R + Orange + Pinkish + White
  colorPals[42].add(colors[2]).add(colors[24]).add(colors[7]).add(colors[25]); // G + Yellowish Green + Turquoise + White
  colorPals[43].add(colors[4]).add(colors[26]).add(colors[7]).add(colors[27]); // B + Purple + Greensish Blue + White

  colorPals[44].add(colors[16]).add(colors[17]).add(colors[19])  // Orange + Pinkish + Turquoise 
               .add(colors[18]).add(colors[20]).add(colors[21]); // + Yellowish Green + Purple + Greensish Blue
  
  colorPals[45].add(colors[22]).add(colors[23]).add(colors[25])  // Orange + Pinkish + Turquoise 
               .add(colors[24]).add(colors[26]).add(colors[27]); // + Yellowish Green + Purple + Greensish Blue

  colorPals[46].add(colors[16]).add(colors[7]).add(colors[17]).add(colors[7]).add(colors[19]).add(colors[7])  // Orange + Pinkish + Turquoise 
               .add(colors[18]).add(colors[7]).add(colors[20]).add(colors[7]).add(colors[21]); // + Yellowish Green + Purple + Greensish Blue
  
  colorPals[47].add(colors[22]).add(colors[7]).add(colors[23]).add(colors[7]).add(colors[25]).add(colors[7])  // Orange + Pinkish + Turquoise 
               .add(colors[24]).add(colors[7]).add(colors[26]).add(colors[7]).add(colors[27]); // + Yellowish Green + Purple + Greensish Blue
};

/** The size of the Patterns buffer **/
const uint16_t NR_PATTERNS = 256;

/** The position(s) used for dynamic patterns **/
const uint16_t PAT_BUF = NR_PATTERNS - 1;

/** The Patterns buffer **/
Pattern* patterns[NR_PATTERNS];

/** Initializes pre-defined patterns */
void initPatterns() {
  // Blink patterns (100%)
  patterns[0] = new Pattern(4);
  patterns[0]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_OFF, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_OFF, 1000);

  patterns[1] = new Pattern(4);
  patterns[1]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON, 1000);

  patterns[2] = new Pattern(4);
  patterns[2]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON_FAST_END, 1000);

  patterns[3] = new Pattern(4);
  patterns[3]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_END, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON_FAST_START, 1000);

  // Blink patterns (15%) * for single color
  patterns[4] = new Pattern(4);
  patterns[4]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_OFF, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_OFF, 1000);

  patterns[5] = new Pattern(4);
  patterns[5]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_ON, 1000);

  patterns[6] = new Pattern(4);
  patterns[6]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_ON_FAST_END, 1000);

  patterns[7] = new Pattern(4);
  patterns[7]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_END, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_ON_FAST_START, 1000);

  // Blink patterns (15% + 0%) * for single color
  patterns[8] = new Pattern(8);
  patterns[8]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_OFF, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_OFF, 1000)
    .addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_OFF, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_OFF, 1000);

  patterns[9] = new Pattern(8);
  patterns[9]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_ON, 1000)
    .addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON, 1000);

  patterns[10] = new Pattern(8);
  patterns[10]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_ON_FAST_END, 1000)
    .addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON_FAST_END, 1000);

  patterns[11] = new Pattern(8);
  patterns[11]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_END, 1000)
    .addColor(Color::create(0.15f, 0.15f, 0.15f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON_FAST_END, 1000);

  // Blink, 1 of 3 LED-s
  patterns[12] = new Pattern(4);
  patterns[12]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_OFF, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_OFF, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_OFF, 1000);
    
  patterns[13] = new Pattern(4);
  patterns[13]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON, 1000);

  // Solid (100%)
  patterns[14] = new Pattern(2);
  patterns[14]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_OFF, 1000);

  patterns[15] = new Pattern(2);
  patterns[15]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON, 1000);

  patterns[16] = new Pattern(4);
  patterns[16]->addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_OFF, 100000);

  // Blink, 1 of 3 LED-s
  patterns[17] = new Pattern(4);
  patterns[17]->addColor(Color::create(1.0f, 1.0f, 1.0f), FADE_ON_FAST_END, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON_FAST_START, 1000)
    .addColor(Color::create(0.0f, 0.0f, 0.0f), FADE_ON_FAST_START, 1000);


  patterns[PAT_BUF] = new Pattern(256);
  patterns[PAT_BUF-1] = new Pattern(256);
  patterns[PAT_BUF-2] = new Pattern(256);
  patterns[PAT_BUF-3] = new Pattern(256);
}

class Preset {
  public:
    uint8_t patNr;
    uint8_t colPalNr;
    uint8_t interleave;
    float intensity;
    float speed;

    Preset() : patNr(0), colPalNr(0), interleave(0), intensity(1.0), speed(1.0) {}

    Preset(uint8_t patNr, uint8_t colPalNr, float intensity, float speed, uint8_t interleave)
      : patNr(patNr), colPalNr(colPalNr), interleave(interleave), intensity(intensity), speed(speed) {}
};

const uint16_t NR_PRESETS = 1024;

Preset presets[NR_PRESETS];

void initPresets() {  
  uint8_t defColPals = 48;
  float speed = 1.5; // faster
  float intensity = 1.0; // max

  // blink + fade (all palettes)
  uint16_t pattern = 1;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  // solid + fade (all palettes)
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[50 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  // RGB interleaved
  presets[97] = Preset(16, 0, intensity, speed, 0);
  presets[98] = Preset(0, 0, intensity, speed, 2);
  presets[99] = Preset(14, 0, intensity, speed, 2);

  // blink + fade (all palettes) + interleaved
  pattern = 1;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[100 + palIdx] = Preset(pattern, palIdx, intensity, speed, 2);
  }

  // solid + fade (all palettes) + interleaved
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[150 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // blink + no fade (all palettes)
  pattern = 0;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[200 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  // solid + no fade (all palettes)
  pattern = 14;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[250 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  // blink + no fade (all palettes) + interleaved
  pattern = 0;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[300 + palIdx] = Preset(pattern, palIdx, intensity, speed, 2);
  }

  // solid + fade (all palettes) + interleaved
  pattern = 14;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[350 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // slower
  speed = 1.0;
  
  // blink + fade (all palettes) + interleaved
  pattern = 1;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[400 + palIdx] = Preset(pattern, palIdx, intensity, speed, 2);
  }

  // solid + fade (all palettes) + interleaved
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[450 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // blink + fade (all palettes) + interleaved
  pattern = 1;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[500 + palIdx] = Preset(pattern, palIdx, intensity, speed, 2);
  }

  // solid + fade (all palettes) + interleaved
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[550 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // really slow + dim
  speed = 0.5;
  intensity = 0.5;
  
  // blink + fade (all palettes)
  pattern = 1;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[600 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  // solid + fade (all palettes)
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[650 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  // blink + fade (all palettes) + interleaved
  pattern = 1;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[700 + palIdx] = Preset(pattern, palIdx, intensity, speed, 2);
  }

  // solid + fade (all palettes) + interleaved
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[750 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // faster + 1 of 3 leds
  speed = 1.5;
  intensity = 0.5;

  // 1 of 3 leds + no fade (all palettes)
  pattern = 12;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[800 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // 1 of 3 leds + fade (all palettes)
  pattern = 13;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[850 + palIdx] = Preset(pattern, palIdx, intensity, speed, 1);
  }

  // slowish
  speed = 1.0;
  intensity = 0.3;
  
  // solid + fade (all palettes)
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[900 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

  speed = 1.0;
  intensity = 0.1;
  
  // solid + fade (all palettes)
  pattern = 15;
  for (uint16_t palIdx = 0; palIdx < defColPals; palIdx++) {
    presets[900 + palIdx] = Preset(pattern, palIdx, intensity, speed, 0);
  }

}

/** RGB LED-s **/
class RGBLed {
  public:
    PWMChan red;
    PWMChan green;
    PWMChan blue;

    RGBLed(PWMChan red, PWMChan green, PWMChan blue)
      : red(red), green(green), blue(blue) { }

    void set(const Color &color) {
      this->red.set(color.red);
      this->green.set(color.green);
      this->blue.set(color.blue);
    }
};

RGBLed LED1(PWMChan(0, 0), PWMChan(0, 1), PWMChan(0, 2));
RGBLed LED2(PWMChan(1, 0), PWMChan(1, 1), PWMChan(1, 2));
RGBLed LED3(PWMChan(2, 0), PWMChan(2, 1), PWMChan(2, 2));

/** PATTERN PLAYER **/

/** Pattern Player. Responsible for timing, and calculating the
  * displayed colors based on a pattern and the settings. */
class PatternPlay {
  public:
    Pattern *pattern;
    Color color;
    uint8_t fadeMode = 0;
    Color curColor;
    Color nextColor;
    uint16_t idx;
    uint64_t startTs = 0;
    uint64_t currColorRelTs = 0;
    uint64_t nextColorRelTs = 0;
    uint64_t relTime = 0;
    float intensity = 0.25;
    float speed = 1.0;

    PatternPlay(): pattern(NULL), idx(-1) { }

    PatternPlay(Pattern *pattern) : pattern(NULL), idx(-1) {
      setPattern(pattern, 0);
    }
    
    void setPattern(Pattern *pattern, uint8_t startIdx) {
      this->pattern = pattern;
      this->idx = startIdx % pattern->size - 1;
      this->startTs = 0;
      this->currColorRelTs = 0;
      this->nextColorRelTs = 0;
      advanceColor();
    }

    void restart(uint8_t startIdx) {
      this->setPattern(this->pattern, startIdx);
    }

    void tick(uint64_t ts) {
      if (this->startTs == 0) {
        this->startTs = ts;
      }

      this->relTime = ts - this->startTs;

      if (relTime > nextColorRelTs) {
        advanceColor();
      }

      this->color = fadeColors();
    }

    Color fadeColors() {
      if (this->fadeMode == FADE_OFF) {
        return this->curColor;
      }

      uint64_t colTime = this->relTime - this->currColorRelTs;
      float prog = (float) colTime / (this->nextColorRelTs - this->currColorRelTs);
      if (this->fadeMode == FADE_ON_FAST_START) {
        prog = prog * prog;
      } else if (this->fadeMode == FADE_ON_FAST_END) {
        prog = 1 - (1 - prog) * (1 - prog);
      }
      return this->fade(this->curColor, this->nextColor, prog);
    }

    void advanceColor() {
      this->idx = this->getNextIdx();
      this->currColorRelTs = this->nextColorRelTs;
      this->nextColorRelTs = this->currColorRelTs + (float) this->pattern->delaysMs[this->idx] / this->speed;
      this->curColor = fade(this->pattern->colors[this->idx], this->intensity);
      this->fadeMode = this->pattern->fades[this->idx];

      this->nextColor = fade(this->pattern->colors[getNextIdx()], this->intensity);
    }

    uint16_t getNextIdx() {
        auto nextIdx = this->idx + 1;
        if (nextIdx >= this->pattern->size) {
          nextIdx = 0;
        }
        return nextIdx;
    }

    Color fade(Color left, Color right, float perc) {
      return Color(
        this->fade(left.red, right.red, perc),
        this->fade(left.green, right.green, perc),
        this->fade(left.blue, right.blue, perc));
    }

    Color fade(Color left, float perc) {
      return Color(
        this->fade(0, left.red, perc),
        this->fade(0, left.green, perc),
        this->fade(0, left.blue, perc));
    }

    uint16_t fade(uint16_t left, uint16_t right, float perc) {
      return (1.0 - perc) * left + perc * right;
    }

    Color getColor() {
      return this->color;
    }

    void setIntensity(float value) {
      this->intensity = value;
      this->curColor = fade(this->pattern->colors[this->idx], this->intensity);
      this->nextColor = fade(this->pattern->colors[getNextIdx()], this->intensity);
    }

    void setSpeed(float newSpeed) {
      float prog = ((float) this->relTime - this->currColorRelTs) / ((float) this->nextColorRelTs - this->currColorRelTs);
      this->speed = newSpeed;
      this->curColor = this->color;
      this->currColorRelTs = this->relTime;
      this->nextColorRelTs = this->currColorRelTs + (float) this->pattern->delaysMs[this->idx] * (1.0 - prog) / this->speed;
    }
};

/** SETTINGS **/

void bleSettingsUpdated();

class Settings {
  public:
    float intensity = 0.25;
    float speed = 1.00;
    uint16_t activeColPal[3] = {0, 0, 0};
    uint16_t activePat[3] = {0, 0, 0};
    bool applyColPal[3] = { false, false, false };
    uint8_t interleave = 0;
    bool sleepMode = false;

    PatternPlay patternPlays[3];

    void selectPreset(uint16_t prsNr) {
      if (prsNr >= NR_PRESETS) return;
      Preset preset = presets[prsNr];
      this->sleepMode = false;
      this->selectColorPalette(preset.colPalNr);
      this->selectPattern(preset.patNr, 0, true, preset.interleave);
      this->setIntensity(preset.intensity);
      this->setSpeed(preset.speed);
    }
    void setSpeed(float value) {
      if (value < 0.0) return;
      this->sleepMode = false;
      this->speed = value;
      this->patternPlays[0].setSpeed(value);
      this->patternPlays[1].setSpeed(value);
      this->patternPlays[2].setSpeed(value);
    }

    void setIntensity(float value) {
      if ((value > 1.0) || (value < 0.0)) return;
      if (value == 0.0) { 
        this->sleep();
        return; 
      }
      this->sleepMode = false;
      this->intensity = value;
      this->patternPlays[0].setIntensity(value);
      this->patternPlays[1].setIntensity(value);
      this->patternPlays[2].setIntensity(value);      
    }

    void selectColor(uint16_t cnr) {
      if (cnr >= NR_COLORS) return;
      this->sleepMode = false;
      colorPals[COLPAL_BUF].reset();
      colorPals[COLPAL_BUF].add(colors[cnr]);
      selectColorPalette(COLPAL_BUF, 0);
    }

    void selectColorPalette(uint16_t cpnr) {
      this->selectColorPalette(cpnr, 0);
    }

    void selectColorPalette(uint16_t cpnr, uint8_t led) {
      if (cpnr >= NR_COLPALS) return;      
      this->sleepMode = false;

      bool allLeds = (led == 0 || led > 3);

      if (allLeds) {
        this->activeColPal[0] = cpnr;
        this->activeColPal[1] = cpnr;
        this->activeColPal[2] = cpnr;
        this->applyColPal[0] = true;
        this->applyColPal[1] = true;
        this->applyColPal[2] = true;
        applyPatterns();

      } else {
        this->activeColPal[led - 1] = cpnr;
        this->applyColPal[led - 1] = true;
        applyPatterns();
      }

      bleSettingsUpdated();
    }

    void selectPattern(uint16_t pnr) {
      selectPattern(pnr, 0, this->applyColPal[0], this->interleave);
    }

    void selectPattern(uint16_t pnr, uint8_t led, bool applyColP, uint8_t interleave) {
      if (patterns[pnr] == NULL) return;
      this->sleepMode = false;

      bool allLeds = (led == 0 || led > 3);

      if (allLeds) {
        this->activePat[0] = pnr;
        this->activePat[1] = pnr;
        this->activePat[2] = pnr;
        this->applyColPal[0] = applyColP;
        this->applyColPal[1] = applyColP;
        this->applyColPal[2] = applyColP;
        this->interleave = interleave;
        applyPatterns();

      } else {
        this->activePat[led - 1] = pnr;
        this->applyColPal[led - 1] = applyColP;
        this->interleave = interleave;
        applyPatterns();
      }

      bleSettingsUpdated();
    }

    void applyPatterns() {
      if ((activePat[0] == activePat[1]) && (activePat[1] == activePat[2])) {
        applyColP(0, PAT_BUF);
        this->patternPlays[0].setPattern(getPattern(0, PAT_BUF), 0);
        this->patternPlays[1].setPattern(getPattern(1, PAT_BUF), interleave);
        this->patternPlays[2].setPattern(getPattern(2, PAT_BUF), 2 * interleave);

      } else {
        applyColP(0, PAT_BUF - 1);
        applyColP(0, PAT_BUF - 2);
        applyColP(0, PAT_BUF - 3);
        this->patternPlays[0].setPattern(getPattern(0, PAT_BUF - 1), 0);
        this->patternPlays[1].setPattern(getPattern(1, PAT_BUF - 2), interleave);
        this->patternPlays[2].setPattern(getPattern(2, PAT_BUF - 3), 2 * interleave);
      }
    }

    void applyColP(uint8_t ledIdx, uint16_t bufIdx) {
      patterns[bufIdx]->apply(patterns[activePat[ledIdx]], colorPals[activeColPal[ledIdx]]);
    }

    Pattern* getPattern(uint8_t ledIdx, uint16_t bufIdx) {
      uint16_t patIdx = applyColPal[ledIdx] ? bufIdx: activePat[ledIdx];
      return patterns[patIdx];
    }

    void timeSync(uint16_t delayMs) {
      delay(delayMs);
      this->patternPlays[0].restart(0);
      this->patternPlays[1].restart(this->interleave);
      this->patternPlays[2].restart(2 * this->interleave);
    }

    void sleep() {
      //this->selectPattern(16); // off
      this->sleepMode = true;
    }
};

/** The global Settings instance **/
Settings settings;


/** BLUETOOTH LOW ENERGY (BLE) **/

BLEPeripheral blePeripheral = BLEPeripheral();

BLEService bleLedService = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLEDescriptor bleLedServiceNameDescriptor = BLEDescriptor("2901", "Snowflake LED");

BLEUnsignedShortCharacteristic bleIntenstiyCharacteristic = BLEUnsignedShortCharacteristic("19b111111e8f2537e4f6cd104768a1214", BLERead | BLEWrite);
BLEDescriptor bleIntenstiyCharacteristicNameDescriptor = BLEDescriptor("2901", "Intensity");

BLEUnsignedShortCharacteristic bleSpeedCharacteristic = BLEUnsignedShortCharacteristic("19b122221e8f2537e4f6cd104768a1214", BLERead | BLEWrite);
BLEDescriptor bleSpeedCharacteristicNameDescriptor = BLEDescriptor("2901", "Speed");

BLEUnsignedShortCharacteristic bleColorCharacteristic = BLEUnsignedShortCharacteristic("19b133331e8f2537e4f6cd104768a1214", BLERead | BLEWrite);
BLEDescriptor bleColorCharacteristicNameDescriptor = BLEDescriptor("2901", "Color");

BLEUnsignedShortCharacteristic bleColorPaletteCharacteristic = BLEUnsignedShortCharacteristic("19b144441e8f2537e4f6cd104768a1214", BLERead | BLEWrite);
BLEDescriptor bleColorPaletteCharacteristicNameDescriptor = BLEDescriptor("2901", "Color Palette");

BLEUnsignedShortCharacteristic blePatternCharacteristic = BLEUnsignedShortCharacteristic("19b155551e8f2537e4f6cd104768a1214", BLERead | BLEWrite);
BLEDescriptor blePatternCharacteristicNameDescriptor = BLEDescriptor("2901", "Pattern");

BLEUnsignedShortCharacteristic blePresetCharacteristic = BLEUnsignedShortCharacteristic("19b177771e8f2537e4f6cd104768a1214", BLERead | BLEWrite);
BLEDescriptor blePresetCharacteristicNameDescriptor = BLEDescriptor("2901", "Preset");

BLEUnsignedShortCharacteristic bleResetSleepTimeCharacteristic = BLEUnsignedShortCharacteristic("19b166661e8f2537e4f6cd104768a1214", BLEWrite);
BLEDescriptor bleResetSleepTimeCharacteristicNameDescriptor = BLEDescriptor("2901", "Reset / Sleep / Time Sync");

BLESerial bleSerial = BLESerial(&blePeripheral);

/** BLE connected event handler */
void blePeripheralConnectHandler(BLECentral& central) {
  settings.sleepMode = false;
}

/** BLE disconnected event handler */
void blePeripheralDisconnectHandler(BLECentral& central) {}

/** Updates BLE characteristic values when settings are updated */
void bleSettingsUpdated() {
  bleIntenstiyCharacteristic.setValue(settings.intensity * 65536.0);
  bleSpeedCharacteristic.setValue(settings.speed * 655.360);
  bleColorPaletteCharacteristic.setValue(settings.activeColPal[0]);
  blePatternCharacteristic.setValue(settings.activePat[0]);
}

/** BLE intensity characteristic handler */
void bleIntenstiyCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = bleIntenstiyCharacteristic.value();
  settings.setIntensity(1.0 * value / 65536);
  bleIntenstiyCharacteristic.setValue(settings.intensity * 65536);
}

/** BLE speed characteristic handler */
void bleSpeedCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = bleSpeedCharacteristic.value();
  settings.setSpeed(1.0 * value / 655.360);
  bleSpeedCharacteristic.setValue(settings.speed * 655.360);
}

/** BLE color characteristic handler */
void bleColorCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = bleColorCharacteristic.value();
  settings.selectColor(value);
  bleColorCharacteristic.setValue(value);
}

/** BLE color palette characteristic handler */
void bleColorPaletteCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = bleColorPaletteCharacteristic.value();
  settings.selectColorPalette(value);
  bleColorPaletteCharacteristic.setValue(value);
}

/** BLE pattern characteristic handler */
void blePatternCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = blePatternCharacteristic.value();
  settings.selectPattern(value);
  blePatternCharacteristic.setValue(value);
}

/** BLE preset characteristic handler */
void blePresetCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = blePresetCharacteristic.value();
  settings.selectPreset(value);
  blePresetCharacteristic.setValue(value);
}

/** BLE Reset / Sleep / Time Sync characteristic handler */
void bleResetSleepTimeCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  short value = bleResetSleepTimeCharacteristic.value();
  if (value == 0) {
    // initiate MCU reset
    NVIC_SystemReset();

  } else if (value == 1) {
    // sleep
    //settings.sleep(); // not relaible (yet)
    NVIC_SystemReset();

  } else {
    // time sync
    settings.timeSync(value);
  }
}

/** BLE Serial Command Handler */
class BLECommands {
  public:
    BLESerial &bleSerial;
    Settings &settings;
    uint8_t bytes[1024];
    char token[32];
    uint16_t pos = 0;
    uint16_t idx = 0;
    uint64_t lastCharTime = 0;

    uint64_t lastBleSend = 0;

    BLECommands(BLESerial &bleSerial, Settings &settings) : bleSerial(bleSerial), settings(settings) {}

    void tick(uint64_t relTime) {
      if (!bleSerial) return;

      int byte;
      while (idx < 1024 && (byte = bleSerial.read()) > 0) {
        if (byte == '\n') {
          process();
        } else {
          this->bytes[idx] = (uint8_t) byte; idx++;
          this->lastCharTime = relTime;
        }
      };

      if (idx >= 1024 || (lastCharTime != 0 && relTime - lastCharTime > 1000)) {
        process();
      }
    }

    void blewrchr(char msg) {
      if (!bleSerial) return;
      bleSerial.write(msg);
      bleSerial.flush();
    }

    void blewr(char *msg) {
      if (!bleSerial) return;
      uint64_t time = millis();
      if (time - lastBleSend < 50) {
        delay(50 - (time - lastBleSend));
      }
      bleSerial.write(msg);
      bleSerial.flush();
      lastBleSend = millis();
    }

    void blewrf(const char* format, ...) {
      if (!bleSerial) return;
      char buffer[128];
      va_list args; va_start(args, format);
      vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      blewr(buffer);
    }

    void process() {
      if (this->idx == 0) {
        return;
      }

      this->pos = 0;

      // read command
      if (!readToken()) {
        clear();
        return;
      }

      if (0 == strcmp(this->token, "INT")) {
        blewr(">INT");
        setIntensity();

      } else if (0 == strcmp(this->token, "SPD")) {
        blewr(">SPD");
        setSpeed();

      } else if (0 == strcmp(this->token, "CLP")) {
        blewr(">CLP");
        selectColorPalette();

      } else if (0 == strcmp(this->token, "PAT")) {
        blewr(">PAT");
        selectPattern();

      } else if (0 == strcmp(this->token, "PRS")) {
        blewr(">PRS");
        selectPreset();

      } else if (0 == strcmp(this->token, "COLC")) {
        blewr(">COLC");
        createColor();

      }  else if (0 == strcmp(this->token, "CLPC")) {
        blewr(">CLPC");
        createColorPalette();

      } else if (0 == strcmp(this->token, "CLPA")) {
        blewr(">CLPA");
        appendColorPalette();

      } else if (0 == strcmp(this->token, "PATC")) {
        blewr(">PATC");
        createPattern();

      } else if (0 == strcmp(this->token, "PATA")) {
        blewr(">PATA");
        appendPattern();

      } else if (0 == strcmp(this->token, "PRSC")) {
        blewr(">PRSC");
        createPreset();

      } else if (0 == strcmp(this->token, "GINT")) {
        blewr(">GINT");
        sendIntensity();

      } else if (0 == strcmp(this->token, "GSPD")) {
        blewr(">GSPD");
        sendSpeed();

      } else if (0 == strcmp(this->token, "GCOL")) {
        blewr(">GCOL");
        sendColor();

      } else if (0 == strcmp(this->token, "GCLP")) {
        blewr(">GCLG");
        sendColorPalette();

      } else if (0 == strcmp(this->token, "GPAT")) {
        blewr(">GPAT");
        sendPattern();

      } else if (0 == strcmp(this->token, "GPRS")) {
        blewr(">GPRS");
        sendPreset();

      } else if (0 == strcmp(this->token, "RST")) {
        blewr(">RST");
        reset();

      } else if (0 == strcmp(this->token, "TSNC")) {
        blewr(">TSNC");
        timeSync();
      
      } else if (0 == strcmp(this->token, "SLP")) {
        blewr(">SLP");
        sleep();

      } else {
        bleSerial.write("UNKOWN command");
      }
      clear();

      blewr(">RDY");
    }

    void sendIntensity() {
      blewrf(">INT %.3f", this->settings.intensity);
    }

    void sendSpeed() {
      blewrf(">SPD %.3f", this->settings.speed);
    }

    void sendColor() {
      while (readToken()) {
        uint16_t colNr = atoi(this->token);
        if (colNr >= NR_COLORS) return;
        Color color = colors[colNr];
        blewrf(">COL %d %.2f %.2f %.2f", colNr, (float) color.red / IMAX, (float) color.green / IMAX, (float) color.blue / IMAX);
      }
    }

    void sendColorPalette() {
      if (!readToken()) return;
      uint16_t colPalNr = atoi(this->token);
      if (colPalNr >= NR_COLPALS) return;
      ColorPal colPal = colorPals[colPalNr];
      blewrf(">CLP %d len=%d", colPalNr, colPal.size);
      for (int nr = 0; nr < colPal.size; nr++) {
        Color color = colPal.colors[nr];
        blewrf(">CLP %d %d %.2f %.2f %.2f", colPalNr, nr, (float) color.red / IMAX, (float) color.green / IMAX, (float) color.blue / IMAX);
      }
    }

    void sendPattern() {
      if (!readToken()) return;
      uint16_t patNr = atoi(this->token);
      if (patNr >= NR_PATTERNS) return;
      Pattern *pattern = patterns[patNr];
      blewrf(">PAT %d len=%d", patNr, pattern->size);
      for (int nr = 0; nr < pattern->size; nr++) {
        Color color = pattern->colors[nr];
        uint8_t fade = pattern->fades[nr];
        uint32_t delays = pattern->delaysMs[nr];
        blewrf(">PAT %d %d %.2f %.2f %.2f %d %d", patNr, nr,
          (float) color.red / IMAX, (float) color.green / IMAX, (float) color.blue / IMAX, fade, delays);
      }
    }

    void sendPreset() {
      if (!readToken()) return;
      uint16_t prsNr = atoi(this->token);
      if (prsNr >= NR_PRESETS) return;
      Preset preset = presets[prsNr];
      blewrf(">PRS %d %d %d %.2f %.2f %d", prsNr, preset.patNr, preset.colPalNr, preset.intensity, preset.speed, preset.interleave);
    }

    void setIntensity() {
      if (!readToken()) return;
      float intensity = atof(this->token);
      settings.setIntensity(intensity);
    }

    void setSpeed() {
      if (!readToken()) return;
      float speed = atof(this->token);
      settings.setSpeed(speed);
    }

    void selectColorPalette() {
      if (!readToken()) return;
      uint16_t colPalNr = atoi(this->token);
      uint8_t ledNr = 0;
      if (readToken() && 'L' == this->token[0]) {
        ledNr = atoi(this->token + 1);
        if (ledNr > 3) ledNr = 0;
      }
      blewrchr('S'); blewrchr('0' + (char) (colPalNr/10));blewrchr('0' + (char) (colPalNr % 10));
      settings.selectColorPalette(colPalNr, ledNr);
    }

    void selectPattern() {
      if (!readToken()) return;
      uint16_t pnr = atoi(this->token);
      if (pnr >= NR_PATTERNS) return;
      uint8_t ledNr = 0;
      uint8_t interleave = 0;
      bool applyColPal = true;
      while (readToken()) {
        if ('L' == this->token[0]) {
          ledNr = atoi(this->token + 1);
          if (ledNr > 3) ledNr = 0;
        } else if ('I' == this->token[0]) {
          interleave = atoi(this->token + 1);
        } else if ('C' == this->token[0]) {
          applyColPal = 1 == atoi(this->token + 1);
        } else {
          break;
        }
      }
      settings.selectPattern(pnr, ledNr, applyColPal, interleave);
    }

    void selectPreset() {
      if (!readToken()) return;
      uint16_t prsNr = atoi(this->token);
      if (prsNr >= NR_PRESETS) return;
      settings.selectPreset(prsNr);
    }

    void createColor() {
      if (!readToken()) return;
      uint16_t cnr = atoi(this->token);
      if (cnr >= 1024) return;
      if (!readToken()) return;
      float red = atof(this->token);
      if (!readToken()) return;
      float green = atof(this->token);
      if (!readToken()) return;
      float blue = atof(this->token);
      colors[cnr] = Color(red * IMAX, green * IMAX, blue * IMAX);
    }

    void createPattern() {
      if (!readToken()) return;
      uint16_t pnr = atoi(this->token);
      if (pnr >= NR_PATTERNS) return;
      if (patterns[pnr] == NULL) {
        patterns[pnr] = new Pattern(256);
      }
      patterns[pnr]->reset();
      processPattern(pnr);
    }

    void appendPattern() {
      if (!readToken()) return;
      uint16_t pnr = atoi(this->token);
      if (pnr >= NR_PATTERNS) return;
      if (patterns[pnr] == NULL) return;
      processPattern(pnr);
    }

    void processPattern(uint16_t pnr) {
      Color color((uint16_t) 0, 0, 0);
      uint8_t fade = FADE_OFF;
      uint32_t delay = 1000;
      uint8_t parseState = 0;

      while (true) {
        if (!readToken()) break;
        if (this->token[0] == 'C') {
          if (parseState != 0) {
            patterns[pnr]->addColor(color, fade, delay);
            fade = FADE_OFF;
            delay = 1000;
          }
          auto cnr = atoi(this->token + 1);
          if (cnr > NR_COLORS) return;
          color = colors[cnr];
          parseState = 1;
        } else if (parseState == 1) {
          fade = atoi(this->token);
          parseState = 2;
        } else if (parseState == 2) {
          delay = atoi(this->token);
          parseState = 3;
        } else { break; }
      }
      if (parseState != 0) {
        patterns[pnr]->addColor(color, fade, delay);
      }

      bleSerial.flush();
    }

    void createColorPalette() {
      if (!readToken()) return;
      uint16_t cpnr = atoi(this->token);
      if (cpnr >= NR_COLPALS) return;
      colorPals[cpnr].reset();
      processColorPalette(cpnr);
    }

    void appendColorPalette() {
      if (!readToken()) return;
      uint16_t cpnr = atoi(this->token);
      if (cpnr >= NR_COLPALS) return;
      processColorPalette(cpnr);
    }

    void processColorPalette(uint16_t pnr) {
      while (true) {
        if (!readToken()) return;
        if (this->token[0] == 'C') {
          char cnr = atoi(this->token + 1);
          if (cnr > NR_COLORS) return;
          Color color = colors[cnr];
          colorPals[pnr].add(color);
        } else { return; }
      }
    }

    void createPreset() {
      if (!readToken()) return;
      uint16_t prsNr = atoi(this->token);
      if (prsNr >= NR_PRESETS) return;
      if (!readToken()) return;
      uint16_t patNr = atoi(this->token);
      if (patNr >= NR_PATTERNS) return;
      if (!readToken()) return;
      uint16_t colPalNr = atoi(this->token);
      if (colPalNr >= NR_COLPALS) return;
      if (!readToken()) return;
      float intensity = atof(this->token);
      if ((intensity > 1.0) || (intensity < 0.0)) return;
      if (!readToken()) return;
      float speed = atof(this->token);
      if (speed < 0.0) return;
      uint16_t interleave = 0;
      if (readToken()) {
        interleave = atoi(this->token);
      }
      presets[prsNr] = Preset(patNr, colPalNr, intensity, speed, interleave);
    }

    void reset() {
      // initiate MCU reset
      NVIC_SystemReset();
    }

    void timeSync() {
      uint16_t delayMs = 0;
      if (readToken()) {
        delayMs = atoi(this->token);
      }
      settings.timeSync(delayMs);
    }

    void sleep() {
      settings.sleep();
    }

    void clear() {
      this->idx = 0;
      this->lastCharTime = 0;
    }

    bool readToken() {
      uint8_t tpos = 0;
      while ((this->pos < this->idx) && tpos < 31) {
        char nextChar = this->bytes[this->pos];
        this->pos++;

        if (nextChar == ' ' || nextChar == '\n') {
          break;
        }

        this->token[tpos] = nextChar;
        tpos++;
      }

      this->token[tpos] = 0;
      return tpos > 0;
    }

};

/** The global BLECommands instance */
BLECommands bleCommands(bleSerial, settings);

void initBLE() {
  // BLE advertised local name and service UUID  
  blePeripheral.setLocalName("Snowflake LED");
  blePeripheral.setAdvertisedServiceUuid(bleLedService.uuid());
  //blePeripheral.setAdvertisingInterval(500); // 0.5 sec

  // BLE Service
  blePeripheral.addAttribute(bleLedService);
  blePeripheral.addAttribute(bleLedServiceNameDescriptor);

  // BLE Characteristics
  blePeripheral.addAttribute(bleIntenstiyCharacteristic);
  blePeripheral.addAttribute(bleIntenstiyCharacteristicNameDescriptor);

  blePeripheral.addAttribute(bleSpeedCharacteristic);
  blePeripheral.addAttribute(bleSpeedCharacteristicNameDescriptor);

  blePeripheral.addAttribute(bleColorCharacteristic);
  blePeripheral.addAttribute(bleColorCharacteristicNameDescriptor);

  blePeripheral.addAttribute(bleColorPaletteCharacteristic);
  blePeripheral.addAttribute(bleColorPaletteCharacteristicNameDescriptor);

  blePeripheral.addAttribute(blePatternCharacteristic);
  blePeripheral.addAttribute(blePatternCharacteristicNameDescriptor);

  blePeripheral.addAttribute(blePresetCharacteristic);
  blePeripheral.addAttribute(blePresetCharacteristicNameDescriptor);

  blePeripheral.addAttribute(bleResetSleepTimeCharacteristic);
  blePeripheral.addAttribute(bleResetSleepTimeCharacteristicNameDescriptor);

  // BLE Event Handlers
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // BLE Characteristic Handlers
  bleIntenstiyCharacteristic.setEventHandler(BLEWritten, bleIntenstiyCharacteristicWritten);
  bleSpeedCharacteristic.setEventHandler(BLEWritten, bleSpeedCharacteristicWritten);
  bleColorCharacteristic.setEventHandler(BLEWritten, bleColorCharacteristicWritten);
  bleColorPaletteCharacteristic.setEventHandler(BLEWritten, bleColorPaletteCharacteristicWritten);
  blePatternCharacteristic.setEventHandler(BLEWritten, blePatternCharacteristicWritten);
  blePresetCharacteristic.setEventHandler(BLEWritten, blePresetCharacteristicWritten);
  bleResetSleepTimeCharacteristic.setEventHandler(BLEWritten, bleResetSleepTimeCharacteristicWritten);  

  // BLE Serial
  bleSerial.init();

  // begin initialization
  blePeripheral.begin();
}

// the setup function runs once when you press reset or power the board
void setup() {
  // Init pins & pwm
  initPins();
  initPWM();

  // Init colors, palettes and patterns
  initColors();
  initColorPals();
  initPatterns();
  initPresets();

  // Init BLE
  initBLE();

  // Init DC/DC
  initDCDC();

  // Default pattern an color
  //97 == off
  settings.selectPreset(863);
 
  settings.sleepMode = true;
}

static uint64_t startTime = 0;

static bool oldSleep = true;

// the loop function runs over and over again forever
void loop() {
  // sleep mode
  if (settings.sleepMode) {   
    if (!oldSleep) {
      disablePWM();      
      if (blePeripheral.connected()) {
        while (blePeripheral.connected()) {
          blePeripheral.poll();
        }
        digitalWrite(LED_1_PINS.red, HIGH);
      }
      disablePins();
    }
    oldSleep = true;
    //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    auto err_code = sd_app_evt_wait();    
    if (err_code != NRF_SUCCESS) {            
      settings.selectPreset(1); // red blinking      
    }
    blePeripheral.poll();
    return;
  }

  if (oldSleep) {
    enablePins();
    enablePWM();
  }

  oldSleep = false;

  uint64_t time = millis();
  if (startTime = 0 || time < startTime) {
    startTime = time;
  }

  uint64_t relTime = time - startTime;
  settings.patternPlays[0].tick(relTime);
  settings.patternPlays[1].tick(relTime);
  settings.patternPlays[2].tick(relTime);

  LED1.set(settings.patternPlays[0].getColor());
  LED2.set(settings.patternPlays[1].getColor());
  LED3.set(settings.patternPlays[2].getColor());

  // BLE
  blePeripheral.poll();
  bleSerial.poll();
  bleCommands.tick(relTime);
}