

# tec-IO
# option 1
## TE-14-23
- to talk to the outside world and also accept information from the outside. 
- It is the first interface in TE that brings the possibility of "robotics" to the TEC1 
- The INPUT/OUTPUT MODULE has one input port and two output ports. 
  - input 8 bits (8 lines) 
  - output 16 bits (16 lines)
- 


## Iterate

- 24 port PPI see https://github.com/SteveJustin1963/tec-IO-PPI/
- Shift Registers
https://lagacemichel.com/2022/07/02/using-shift-registers-to-increase-digital-outputs/


<img width="555" height="555" alt="image" src="https://github.com/user-attachments/assets/ae9462eb-8b16-4b9d-bafb-1c0d93bad90e" />

# option 2
# using pc and io running mint on octave
It depends a bit on exactly what “simple” means for your needs (number of pins, speed, voltage levels, analog vs digital, latency, library support, etc.), but yes — there *are* USB GPIO (or “USB to general purpose I/O”) modules that can work nicely with Octave (or any PC) via serial/USB protocols.

Here are some tips + suggestions + caveats, and a few product ideas:

---

## What to look for / requirements

Before picking one, make sure:

1. **Interface / protocol** — Ideally the board appears as a USB-to-serial (CDC), or a USBTMC / USB HID, or some standard interface, so you can drive it from Octave (via instrument-control, serial, etc.). Octave’s *instrument-control* package supports USBTMC among other interfaces. ([octave.sourceforge.io][1])
2. **Voltage / logic levels** — Do you need 3.3 V, 5 V, or both? Make sure the module supports your target levels (or is suitable for level-shifting).
3. **Number of GPIOs / I/O flexibility** — How many pins do you need? Do you need them individually configurable (input/output, pull-ups, etc.)?
4. **Speed / latency** — Over USB, toggling a pin might not be super fast, so if you need high-speed toggling or precise timing, check how fast the interface is.
5. **Driver / library support** — It’s a big plus if there’s sample code, or it uses a common chip (like FT232H, MCP2221, etc.) so you can reuse existing libraries.
6. **Analog inputs / extra features** (if needed) — Some USB GPIO modules also provide analog input channels, which is a bonus if you want to do sensor reading, not just digital I/O.

---

## Some example boards / modules you can buy

Here are a few USB GPIO (or USB to I/O) devices worth considering, which are relatively simple and have decent support:

### [Adafruit MCP2221A USB‑GPIO Breakout]()

#### well-known chip & library support

*$11.23*

### [Generic 10‑pin USB to GPIO module]()

#### cheap generic module

*$9.39*

### [RP2350 USB GPIO mini board]()

#### modern microcontroller board

*$7.59*

### [Arduino UNO R3 (as USB I/O)]()

#### versatile and well supported

*$19.99*

Here are the highlights:

| Product                                     | Why it’s interesting / pros & cons                                                                                                                          |
| ------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **[Adafruit MCP2221A USB‑GPIO Breakout]()** | A compact breakout using the MCP2221A chip (USB to I/O + ADC). Good for small I/O needs; has driver/library support.                                        |
| **[Generic 10‑pin USB to GPIO module]()**   | Very cheap/simple module with 10 GPIO pins exposed. Might lack robust documentation but useful for basic tasks.                                             |
| **[RP2350 USB GPIO mini board]()**          | A more modern microcontroller board providing about 15 GPIOs via USB. Good middle ground.                                                                   |
| **[Arduino UNO R3 (as USB I/O)]()**         | Although not a “pure” USB GPIO board, you can load a simple sketch on the Uno to turn it into a USB-serial I/O interface. Very flexible and well supported. |

---

## Recommendation & how I’d use one with Octave

If I were you and wanted something that *just works* without too much fuss, I’d lean toward the **Adafruit MCP2221A USB-GPIO Breakout** or using an **Arduino (UNO or similar)** as a USB-serial bridge for I/O.

### Using Arduino as USB GPIO

* You’d upload a small firmware/sketch in Arduino that listens for commands over serial (e.g. “set pin 7 HIGH”, “read pin 3”, etc.).
* On the PC side, Octave can use the *instrument-control* package or just open the serial port (e.g. `/dev/ttyACM0` or `COMx`) to send those commands and get responses.
* This gives you full control, simple protocol, and plenty of pins.
* The latency is good for many practical use cases (though not for super high-speed switching in the tens of MHz).

### Using MCP2221A or similar

* MCP2221A is a USB-to-UART / I²C / GPIO chip. It is supported by many driver libraries.
* Because it is a dedicated chip, it's smaller, cheaper, and perhaps lower overhead than a full microcontroller.
* But you’re limited to whatever features the chip supports (e.g. limited number of GPIO, ADCs, etc.)

---

## Caveats & things to watch out for

* As I mentioned, USB adds overhead. If your application needs *very precise timing* (e.g. toggling a pin at ~MHz rates), a USB GPIO board might not suffice.
* Ensure the board’s logic voltage matches your external circuits, or use level shifting.
* Make sure the board is well documented, ideally supported on your OS (Windows, Linux, etc.).
* In Octave, you’ll probably use **serial** or **instrument-control** commands to talk to the board.

---

If you like, I can narrow down a few boards *available in Australia / with local shipping*, and even check that they are compatible with Octave (drivers, examples). Do you want me to filter for Australian retailers / local stock?

[1]: https://octave.sourceforge.io/instrument-control/package_doc/?utm_source=chatgpt.com "Top (Octave Instrument Control Toolkit) - SourceForge"


Perfect — since you’re running **Octave on Lubuntu**, you’ve got a lot of flexibility.
Here’s the **best and simplest USB GPIO setup** that works cleanly in that environment:

---

## 🥇 **Option 1: Adafruit MCP2221A USB-to-GPIO/I²C Bridge**

**✅ Best for: Plug-and-play GPIO and I²C over USB in Linux/Octave**

* **Price:** ~ A$15–25 (AU suppliers: Core Electronics, Digi-Key AU, RS Online)
* **Chip:** Microchip MCP2221A
* **GPIOs:** 4 digital pins + ADC inputs
* **Extras:** Built-in I²C master, UART, 3.3 V or 5 V logic
* **Linux driver:** Already built into Lubuntu kernels (appears as `/dev/hidrawX`)
* **Octave access:** Use the *instrument-control* package via HID or serial; or simple Python/CLI utilities callable from Octave (`system("mcp2221ctl …")`)

**Example Octave snippet:**

```octave
system("mcp2221ctl --set-gpio 0=1");  % set GPIO0 HIGH
system("mcp2221ctl --get-gpio 1");    % read GPIO1
```

**Pros:** No microcontroller programming, rock-solid Linux support, very small.
**Cons:** Only 4 GPIO pins.

---

## 🥈 **Option 2: Arduino UNO (or Nano Every) as USB GPIO**

**✅ Best for: Many pins, easy custom protocol**

* **Price:** A$10–25 (clones OK)
* **GPIOs:** ~ 14 digital + 6 analog (5 V logic)
* **How it shows up:** `/dev/ttyACM0` (serial port)
* **Octave access:** `serial()` from the *instrument-control* package

**Simple Octave example:**

```octave
pkg load instrument-control;
s = serial("/dev/ttyACM0");
srl_write(s, "D13 HIGH\n");
```

You’d upload a short Arduino sketch that listens for text commands like
`D13 HIGH` or `A0?` and replies with values.

**Pros:** Lots of pins, cheap, works on any PC.
**Cons:** You write a tiny firmware once (but easy to do).

---

## 🥉 **Option 3: FT232H or FT2232H breakout**

**✅ Best for: Fast GPIO bit-banging, SPI, I²C**

* **Price:** A$25–40
* **Libraries:** libftdi, pyftdi (works on Linux)
* **Octave link:** Call Python scripts from Octave for advanced I/O.
  Great if you later expand into SPI sensors or fast bit streams.

---

## ⚙️ Setup tips for Lubuntu + Octave

1. `sudo apt install octave octave-instrument-control`
2. Plug your device, then check with `dmesg | grep tty` or `ls /dev/hidraw*`.
3. For Arduino-type devices, add your user to the dialout group:
   `sudo usermod -aG dialout $USER` → reboot.
4. Test basic serial or HID commands from Octave before wiring anything.

---

## 🔧 Summary Table

| Device                |  GPIO Pins | Needs Firmware | Interface | Works on Lubuntu | Difficulty | Notes                      |
| :-------------------- | :--------: | :------------: | :-------: | :--------------: | :--------: | :------------------------- |
| **MCP2221A Breakout** |      4     |       No       |  HID/USB  |         ✔        |    Easy    | Best “no-code” option      |
| **Arduino UNO/Nano**  | 14 + 6 ADC |   Yes (once)   |   Serial  |         ✔        |    Easy    | Flexible and cheap         |
| **FT232H Breakout**   |    8–16    |       No       |    USB    |         ✔        |   Medium   | High-speed I/O and SPI/I²C |

---

If you tell me how many GPIO pins and what voltage/speed you need, I can recommend the **exact model and vendor link in Australia**.
Would you like that next?


Exactly 👍 — the **Arduino Nano** (especially the classic ATmega328P or the newer Nano Every) is one of the **cheapest and most capable USB GPIO interfaces** you can use under Lubuntu and Octave.

Here’s why it’s such a strong choice for you:

---

## 🧠 What You Get with the Nano

| Feature            | Details                                                              |
| :----------------- | :------------------------------------------------------------------- |
| **Digital GPIO**   | 14 pins (D0–D13), any configurable as input/output                   |
| **Analog inputs**  | 8 pins (A0–A7), 10-bit ADC                                           |
| **I²C**            | Pins A4 (SDA) and A5 (SCL)                                           |
| **SPI**            | Pins D10 (CS), D11 (MOSI), D12 (MISO), D13 (SCK)                     |
| **UART (Serial)**  | D0/D1 and the same USB port you plug into your PC                    |
| **Logic levels**   | 5 V (3.3 V tolerant on some inputs; use level shifters if needed)    |
| **Power from USB** | Runs directly off your PC’s USB port, no extra power supply          |
| **Price**          | A$6 – A$15 (clones everywhere on eBay, AliExpress, Core Electronics) |

---

## 🪄 Why It Works Great with Octave

* The Nano shows up as a **USB serial port** (`/dev/ttyUSB0` or `/dev/ttyACM0`).
* You can talk to it using **Octave’s `instrument-control` package** (`serial()` interface).
* You can write a **simple firmware sketch** that listens for ASCII commands (e.g., “SET D13 1” or “READ A0”) and responds with readings.

**Example Octave control:**

```octave
pkg load instrument-control;
s = serial("/dev/ttyUSB0", 115200);
srl_write(s, "SET D13 1\n");   % turn on pin 13 (LED)
pause(1);
srl_write(s, "SET D13 0\n");   % turn off
srl_write(s, "READ A0\n");     % ask for analog value
val = str2num(char(srl_read(s, 10)));  % get reply
disp(val);
```

---

## ⚙️ Example Arduino Sketch

Upload this once via the Arduino IDE:

```cpp
void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "SET D13 1") digitalWrite(13, HIGH);
    else if (cmd == "SET D13 0") digitalWrite(13, LOW);
    else if (cmd == "READ A0") {
      int v = analogRead(A0);
      Serial.println(v);
    }
  }
}
```

Now Octave can drive your Nano like a remote GPIO/I²C/SPI adapter.

---

## 🧩 Using SPI and I²C from the Nano

* **I²C:** use Arduino’s `Wire.h` library; your Octave commands can trigger reads/writes to I²C sensors or other chips.
* **SPI:** use Arduino’s `SPI.h` library; you can control devices such as DACs, ADCs, displays, or SD cards.
* The Nano handles the low-level protocol timing; Octave just tells it what to do over serial.

---

## 🚀 Why It’s Ideal for You

* **Cheap** — clones cost under A$10.
* **Simple** — plug & play USB, no special drivers on Lubuntu.
* **Flexible** — GPIO, ADC, PWM, I²C, SPI, UART all available.
* **Expandable** — add modules later (sensors, displays, encoders, etc.).

---

If you’d like, I can show you a ready-to-upload **firmware + Octave driver pair** that gives you commands like `gpio_write(pin,val)` and `i2c_read(addr,len)` directly from Octave.
Would you like me to generate that next?


Yes — absolutely.
That’s one of the big advantages of using a **Nano** (or any Arduino-class board) instead of a fixed-function USB-GPIO dongle.

---

### 🧩 You can run your own code **and** use all the hardware features

When you load your own sketch (firmware) onto the Nano, it becomes a **custom USB device** that:

* Talks to your PC via USB serial (so Octave can send commands),
* **Runs its own logic simultaneously**, and
* Exposes the built-in peripherals of the ATmega328P:

| Feature               | Description               | Example use                                                         |
| :-------------------- | :------------------------ | :------------------------------------------------------------------ |
| **GPIO**              | 14 digital I/O pins       | Toggle LEDs, read switches, drive logic                             |
| **PWM**               | D3, D5, D6, D9, D10, D11  | Output analog-like voltages for motors, dimming LEDs, DAC emulation |
| **ADC**               | A0–A7 (10-bit)            | Measure analog sensors, voltages                                    |
| **SPI**               | D10–D13                   | Talk to ADCs, DACs, displays, SD cards                              |
| **I²C**               | A4 (SDA), A5 (SCL)        | Communicate with sensors, EEPROMs, etc.                             |
| **UART**              | D0/D1 + USB               | Serial link to PC (Octave, terminal, etc.)                          |
| **Timers/interrupts** | 3 timers + pin interrupts | Timing loops, pulse counters, etc.                                  |

---

### ⚙️ How it works in practice

You flash one sketch that:

1. Does any “always-on” background task you want (PWM control, sensor reading, etc.), and
2. Listens on `Serial` for commands from Octave, responding with results or updating variables.

So, you can have a hybrid design:

```cpp
// Example: Nano does local tasks + Octave control
int pwmVal = 0;
void setup() {
  Serial.begin(115200);
  pinMode(9, OUTPUT);
}

void loop() {
  analogWrite(9, pwmVal);   // run PWM all the time

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("PWM ")) pwmVal = cmd.substring(4).toInt();
    else if (cmd == "ADC A0") Serial.println(analogRead(A0));
  }
}
```

Then, in Octave:

```octave
pkg load instrument-control;
s = serial("/dev/ttyUSB0", 115200);
srl_write(s, "PWM 128\n");    % 50% duty cycle
pause(1);
srl_write(s, "ADC A0\n");
val = str2num(char(srl_read(s, 10)));
disp(val);
```

Now the Nano is simultaneously:

* Generating PWM on D9,
* Reading ADC when asked, and
* Responding to Octave over USB.

---

### 💡 You can expand further

* Add your own sensor loops, filters, PID control, or motor drivers inside the sketch.
* Octave just sends commands or collects data when needed.
* The Nano can even **stream sensor data** (e.g., every 100 ms) back to Octave for plotting.
* PWM can act as a **pseudo-DAC** (filtered with an RC network if you need analog voltage).

---

### 🪄 In summary

✅ Yes — you can:

* Run **your own firmware logic** on the Nano,
* Access **GPIO, PWM, ADC, SPI, I²C**,
* Talk to it directly from **Octave on Lubuntu** via USB,
* Combine both sides (local control + PC analysis/plotting).

---

Would you like me to give you a **ready-to-flash Nano firmware + Octave driver script** that includes GPIO, PWM, ADC, SPI, and I²C commands all in one simple protocol?


Perfect — here’s a **complete ready-to-use setup** for your **Arduino Nano ↔ Octave on Lubuntu** link.
It gives you a clean command protocol covering **GPIO, PWM, ADC, SPI, and I²C**, so you can drive the Nano entirely from Octave while it still runs its own logic.

---

# 🧩 1. Upload this firmware to your **Arduino Nano**

**File:** `nano_io_bridge.ino`

```cpp
#include <Wire.h>
#include <SPI.h>

#define MAX_CMD 32
char cmd[MAX_CMD];
int idx = 0;
int pwmVal = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();        // I²C master
  SPI.begin();         // SPI master
  pinMode(13, OUTPUT); // onboard LED
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd[idx] = '\0';
      processCommand(cmd);
      idx = 0;
    } else if (idx < MAX_CMD - 1) {
      cmd[idx++] = c;
    }
  }

  // Example background task
  analogWrite(9, pwmVal);  // PWM output always running
}

void processCommand(char *cmd) {
  // --- GPIO ---
  if (sscanf(cmd, "PINMODE %d %d", &idx, &pwmVal) == 2) {
    pinMode(idx, pwmVal);
    Serial.println("OK");
  }
  else if (sscanf(cmd, "DIGWRITE %d %d", &idx, &pwmVal) == 2) {
    digitalWrite(idx, pwmVal);
    Serial.println("OK");
  }
  else if (sscanf(cmd, "DIGREAD %d", &idx) == 1) {
    Serial.println(digitalRead(idx));
  }

  // --- ADC ---
  else if (sscanf(cmd, "ADC %d", &idx) == 1) {
    Serial.println(analogRead(idx));
  }

  // --- PWM ---
  else if (sscanf(cmd, "PWM %d %d", &idx, &pwmVal) == 2) {
    analogWrite(idx, pwmVal);
    Serial.println("OK");
  }

  // --- I2C (simple one-byte read/write) ---
  else if (strncmp(cmd, "I2CWRITE", 8) == 0) {
    int addr, val;
    if (sscanf(cmd + 8, "%d %d", &addr, &val) == 2) {
      Wire.beginTransmission(addr);
      Wire.write(val);
      Wire.endTransmission();
      Serial.println("OK");
    }
  }
  else if (strncmp(cmd, "I2CREAD", 7) == 0) {
    int addr, n;
    if (sscanf(cmd + 7, "%d %d", &addr, &n) == 2) {
      Wire.requestFrom(addr, n);
      while (Wire.available()) {
        Serial.print(Wire.read());
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  // --- SPI one-byte transaction ---
  else if (strncmp(cmd, "SPI", 3) == 0) {
    unsigned int data;
    if (sscanf(cmd + 3, "%x", &data) == 1) {
      byte res = SPI.transfer((byte)data);
      Serial.println(res, HEX);
    }
  }

  // --- PWM variable example ---
  else if (sscanf(cmd, "PWMS %d", &pwmVal) == 1) {
    // set global PWM value used in background loop
    Serial.println("OK");
  }

  else if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
  }

  else {
    Serial.println("ERR");
  }
}
```

**Flash it** via the Arduino IDE once (select *Tools → Board: “Arduino Nano” → Processor: ATmega328P*).

---

# 💻 2. Octave driver script for Lubuntu

**File:** `nano_io_driver.m`

```octave
pkg load instrument-control;
port = "/dev/ttyUSB0";   % or /dev/ttyACM0 depending on Nano type
baud = 115200;
s = serial(port, baud);

function reply = ar_send(s, txt)
  srl_write(s, [txt "\n"]);
  pause(0.05);
  reply = strtrim(char(srl_read(s, 128)));
endfunction

% ---- examples ----

disp(ar_send(s, "PING"));          % expect PONG

% GPIO
ar_send(s, "PINMODE 13 1");        % set pin 13 output
ar_send(s, "DIGWRITE 13 1");       % LED ON
pause(1);
ar_send(s, "DIGWRITE 13 0");       % LED OFF

% ADC
val = ar_send(s, "ADC 0");         % read A0 (0-1023)
printf("ADC0=%s\n", val);

% PWM
ar_send(s, "PWM 9 128");           % 50% duty PWM on D9

% I2C one-byte read/write
ar_send(s, "I2CWRITE 0x40 0xFF");  % send 0xFF to address 0x40
reply = ar_send(s, "I2CREAD 0x40 2");  % read two bytes

% SPI example
reply = ar_send(s, "SPI 55");      % send 0x55, print returned byte
printf("SPI reply: %s\n", reply);
```

Run this from Octave, and you’ve got a bidirectional bridge.

---

# ⚙️ Notes

* Logic levels: 5 V (use level shifters for 3.3 V devices).
* SPI pins: D10 = CS, D11 = MOSI, D12 = MISO, D13 = SCK.
* I²C pins: A4 = SDA, A5 = SCL.
* PWM outputs: D3, D5, D6, D9, D10, D11.
* ADC: A0–A7.
* Use `sudo usermod -aG dialout $USER` once so Octave can access `/dev/ttyUSB0`.

---

# 🔧 Optional: background logic

You can freely expand the `loop()` with your own control routines —
PID control, sensor averaging, DAC emulation, etc. —
and the Octave link still works because it just listens on `Serial`.

---

Would you like me to extend this sketch to include **streaming ADC data to Octave in real time** (for plotting waveforms or sensor graphs)?

Excellent — here’s the upgraded version of your **Nano firmware + Octave interface**, now supporting **real-time ADC streaming** for live plotting in Octave.

---

## 🧩 1. Flash this Arduino Nano firmware

**File:** `nano_io_bridge_stream.ino`

```cpp
#include <Wire.h>
#include <SPI.h>

#define MAX_CMD 32
char cmd[MAX_CMD];
int idx = 0;
int pwmVal = 0;
bool streaming = false;
int streamPin = A0;
unsigned long lastSample = 0;
unsigned int sampleInterval = 100; // milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin();
  SPI.begin();
  pinMode(13, OUTPUT);
}

void loop() {
  // --- handle commands ---
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd[idx] = '\0';
      processCommand(cmd);
      idx = 0;
    } else if (idx < MAX_CMD - 1) {
      cmd[idx++] = c;
    }
  }

  // --- background PWM loop ---
  analogWrite(9, pwmVal);

  // --- ADC streaming mode ---
  if (streaming && millis() - lastSample >= sampleInterval) {
    lastSample = millis();
    int v = analogRead(streamPin);
    Serial.println(v);
  }
}

void processCommand(char *cmd) {
  if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
  }

  else if (sscanf(cmd, "PWM %d %d", &idx, &pwmVal) == 2) {
    analogWrite(idx, pwmVal);
    Serial.println("OK");
  }

  else if (sscanf(cmd, "ADC %d", &idx) == 1) {
    Serial.println(analogRead(idx));
  }

  else if (sscanf(cmd, "STREAM %d %d", &streamPin, &sampleInterval) == 2) {
    streaming = true;
    Serial.println("STREAM ON");
  }

  else if (strcmp(cmd, "STREAM STOP") == 0) {
    streaming = false;
    Serial.println("STREAM OFF");
  }

  else {
    Serial.println("ERR");
  }
}
```

---

## 💻 2. Octave script to stream and plot in real time

**File:** `nano_stream_plot.m`

```octave
pkg load instrument-control;
port = "/dev/ttyUSB0";    % or /dev/ttyACM0
baud = 115200;
s = serial(port, baud);

function ar_send(s, txt)
  srl_write(s, [txt "\n"]);
  pause(0.05);
endfunction

% Start streaming from A0 every 50 ms
ar_send(s, "STREAM 0 50");

N = 200;                   % number of samples to plot
data = zeros(1, N);
figure(1); clf;
title("Real-time ADC Streaming from Nano");
xlabel("Sample #"); ylabel("ADC Value (0–1023)");

for i = 1:N
  line = strtrim(char(srl_read(s, 20)));
  if !isempty(line)
    val = str2num(line);
    if !isempty(val)
      data = [data(2:end) val];
      plot(data);
      axis([0 N 0 1023]);
      drawnow;
    endif
  endif
endfor

% Stop the stream
ar_send(s, "STREAM STOP");
fclose(s);
```

---

## ⚙️ How it works

1. The Nano streams raw ADC values continuously via USB serial.
2. Octave reads and plots them in real time (like a small oscilloscope).
3. You can change the sampling interval in ms (`STREAM 0 10` = 100 Hz).

---

## 🧪 Example uses

* Plot voltage from a **potentiometer**, **photoresistor**, or **sensor**.
* Observe **PWM signals** (after RC filtering).
* Stream temperature or light readings over time.
* Combine multiple pins later for multi-channel capture.

---

Would you like me to extend this version further — for example:

* **stream two ADC channels at once**,
* or **send timestamped data (ms + value)** for plotting in Octave with real time vs voltage graphs?
