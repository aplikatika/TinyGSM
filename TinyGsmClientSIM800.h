/**
 * @file       TinyGsmClientSIM800.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef TinyGsmClientSIM800_h
#define TinyGsmClientSIM800_h

//#define TINY_GSM_DEBUG Serial
//#define TINY_GSM_USE_HEX

#if !defined(TINY_GSM_RX_BUFFER)
  #define TINY_GSM_RX_BUFFER 64
#endif

#define TINY_GSM_MUX_COUNT 5

#include <TinyGsmCommon.h>

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;

enum SimStatus {
  SIM_ERROR = 0,
  SIM_READY = 1,
  SIM_LOCKED = 2,
};

enum RegStatus {
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class TinyGsmSim800
{

public:

class GsmClient : public Client
{
  friend class TinyGsmSim800;
  typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

public:
  GsmClient() {}

  GsmClient(TinyGsmSim800& modem, uint8_t mux = 1) {
    init(&modem, mux);
  }

  bool init(TinyGsmSim800* modem, uint8_t mux = 1) {
    this->at = modem;
    this->mux = mux;
    sock_available = 0;
    sock_connected = false;
    got_data = false;

    at->sockets[mux] = this;

    return true;
  }

public:
  virtual int connect(const char *host, uint16_t port) {
    TINY_GSM_YIELD();
    rx.clear();
    sock_connected = at->modemConnect(host, port, mux);
    return sock_connected;
  }

  virtual int connect(IPAddress ip, uint16_t port) {
    String host; host.reserve(16);
    host += ip[0];
    host += ".";
    host += ip[1];
    host += ".";
    host += ip[2];
    host += ".";
    host += ip[3];
    return connect(host.c_str(), port);
  }

  virtual void stop() {
    TINY_GSM_YIELD();
    at->sendAT(GF("+CIPCLOSE="), mux);
    sock_connected = false;
    at->waitResponse();
  }

  virtual size_t write(const uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    at->maintain();
    return at->modemSend(buf, size, mux);
  }

  virtual size_t write(uint8_t c) {
    return write(&c, 1);
  }

  virtual int available() {
    TINY_GSM_YIELD();
    if (!rx.size() && sock_connected) {
      // Workaround: sometimes SIM800 forgets to notify about data arrival.
      // TODO: Currently we ping the module periodically,
      // but maybe there's a better indicator that we need to poll
      static unsigned long timeout = 0;
      if (0 < (long)(timeout - millis())) {
        timeout = millis() + 500L;
        got_data = true;
      }
      at->maintain();
    }
    return rx.size() + sock_available;
  }

  virtual int read(uint8_t *buf, size_t size) {
    TINY_GSM_YIELD();
    at->maintain();
    size_t cnt = 0;
    while (cnt < size) {
      size_t chunk = TinyGsmMin(size-cnt, rx.size());
      if (chunk > 0) {
        rx.get(buf, chunk);
        buf += chunk;
        cnt += chunk;
        continue;
      }
      // TODO: Read directly into user buffer?
      at->maintain();
      if (sock_available > 0) {
        at->modemRead(rx.free(), mux);
      } else {
        break;
      }
    }
    return cnt;
  }

  virtual int read() {
    uint8_t c;
    if (read(&c, 1) == 1) {
      return c;
    }
    return -1;
  }

  virtual int peek() { return -1; } //TODO
  virtual void flush() { at->stream.flush(); }

  virtual uint8_t connected() {
    if (available()) {
      return true;
    }
    return sock_connected;
  }
  virtual operator bool() { return connected(); }

  /*
   * Extended API
   */

  String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;

private:
  TinyGsmSim800* at;
  uint8_t       mux;
  uint16_t      sock_available;
  bool          sock_connected;
  bool          got_data;
  RxFifo        rx;
};

class GsmClientSecure : public GsmClient
{
public:
  GsmClientSecure() {}

  GsmClientSecure(TinyGsmSim800& modem, uint8_t mux = 1)
    : GsmClient(modem, mux)
  {}

public:
  virtual int connect(const char *host, uint16_t port) {
    TINY_GSM_YIELD();
    rx.clear();
    sock_connected = at->modemConnect(host, port, mux, true);
    return sock_connected;
  }
};

public:

  TinyGsmSim800(Stream& stream)
    : stream(stream)
  {
    memset(sockets, 0, sizeof(sockets));
  }

  /*
   * Basic functions
   */
  bool begin() {
    return init();
  }

  bool init() {
    if (!testAT()) {
      return false;
    }
    sendAT(GF("&FZ"));  // Factory + Reset
    waitResponse();
    sendAT(GF("E0"));   // Echo Off
    if (waitResponse() != 1) {
      return false;
    }
  
    // PREFERRED SMS STORAGE
    sendAT(GF("+CPMS="), GF("\"SM\""), GF(","), GF("\"SM\""), GF(","), GF("\"SM\""));
    waitResponse();
  sendAT(GF("+CMGF=1")); // SMS set TEXT mode
  waitResponse();
  sendAT(GF("+CSDH=0")); // SMS don't show all header values
  waitResponse();

    getSimStatus();
    return true;
  }

  void setBaud(unsigned long baud) {
    sendAT(GF("+IPR="), baud);
  }

  bool testAT(unsigned long timeout = 10000L) {
    //streamWrite(GF("AAAAA" GSM_NL));  // TODO: extra A's to help detect the baud rate
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      sendAT(GF(""));
      if (waitResponse(200) == 1) {
          delay(100);
          return true;
      }
      delay(100);
    }
    return false;
  }

  void maintain() {
    for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
      GsmClient* sock = sockets[mux];
      if (sock && sock->got_data) {
        sock->got_data = false;
        sock->sock_available = modemGetAvailable(mux);
      }
    }
    while (stream.available()) {
      waitResponse(10, NULL, NULL);
    }
  }

  bool factoryDefault() {
    sendAT(GF("&FZE0&W"));  // Factory + Reset + Echo Off + Write
    waitResponse();
    sendAT(GF("+IPR=0"));   // Auto-baud
    waitResponse();
    sendAT(GF("+IFC=0,0")); // No Flow Control
    waitResponse();
    sendAT(GF("+ICF=3,3")); // 8 data 0 parity 1 stop
    waitResponse();
    sendAT(GF("+CSCLK=0")); // Disable Slow Clock
    waitResponse();
    sendAT(GF("&W"));       // Write configuration
    return waitResponse() == 1;
  }

  String getModemInfo() {
    sendAT(GF("I"));
    String res;
    if (waitResponse(1000L, res) != 1) {
      return "";
    }
    res.replace(GSM_NL "OK" GSM_NL, "");
    res.replace(GSM_NL, " ");
    res.trim();
    return res;
  }

  bool hasSSL() {
    sendAT(GF("+CIPSSL=?"));
    if (waitResponse(GF(GSM_NL "+CIPSSL:")) != 1) {
      return false;
    }
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */

  bool restart() {
    if (!testAT()) {
      return false;
    }
    sendAT(GF("+CFUN=0"));
    if (waitResponse(10000L) != 1) {
      return false;
    }
    sendAT(GF("+CFUN=1,1"));
    if (waitResponse(10000L) != 1) {
      return false;
    }
    delay(3000);
    return init();
  }

  bool poweroff() {
    sendAT(GF("+CPOWD=1"));
    return waitResponse(GF("NORMAL POWER DOWN")) == 1;
  }

  bool radioOff() {
    sendAT(GF("+CFUN=0"));
    if (waitResponse(10000L) != 1) {
      return false;
    }
    delay(3000);
    return true;
  }

  /*
    During sleep, the SIM800 module has its serial communication disabled. In order to reestablish communication
    pull the DRT-pin of the SIM800 module LOW for at least 50ms. Then use this function to disable sleep mode.
    The DTR-pin can then be released again.
  */
  bool sleepEnable(bool enable = true) {
    sendAT(GF("+CSCLK="), enable);
    return waitResponse() == 1;
  }

  /*
   * SIM card functions
   */

  bool simUnlock(const char *pin) {
    sendAT(GF("+CPIN=\""), pin, GF("\""));
    return waitResponse() == 1;
  }

  String getSimCCID() {
    sendAT(GF("+ICCID"));
    if (waitResponse(GF(GSM_NL "+ICCID:")) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  String getIMEI() {
    sendAT(GF("+GSN"));
    if (waitResponse(GF(GSM_NL)) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  SimStatus getSimStatus(unsigned long timeout = 10000L) {
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      sendAT(GF("+CPIN?"));
      if (waitResponse(GF(GSM_NL "+CPIN:")) != 1) {
        delay(1000);
        continue;
      }
      int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK"), GF("NOT INSERTED"));
      waitResponse();
      switch (status) {
      case 2:
      case 3:  return SIM_LOCKED;
      case 1:  return SIM_READY;
      default: return SIM_ERROR;
      }
    }
    return SIM_ERROR;
  }

  RegStatus getRegistrationStatus() {
    sendAT(GF("+CREG?"));
    if (waitResponse(GF(GSM_NL "+CREG:")) != 1) {
      return REG_UNKNOWN;
    }
    streamSkipUntil(','); // Skip format (0)
    int status = stream.readStringUntil('\n').toInt();
    waitResponse();
    return (RegStatus)status;
  }

  String getOperator() {
    sendAT(GF("+COPS?"));
    if (waitResponse(GF(GSM_NL "+COPS:")) != 1) {
      return "";
    }
    streamSkipUntil('"'); // Skip mode and format
    String res = stream.readStringUntil('"');
    waitResponse();
    return res;
  }

  /*
   * Generic network functions
   */

  int getSignalQuality() {
    sendAT(GF("+CSQ"));
    if (waitResponse(GF(GSM_NL "+CSQ:")) != 1) {
      return 99;
    }
    int res = stream.readStringUntil(',').toInt();
    waitResponse();
    return res;
  }

  bool isNetworkConnected() {
    RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

  bool waitForNetwork(unsigned long timeout = 60000L) {
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      if (isNetworkConnected()) {
        return true;
      }
      delay(500);
    }
    return false;
  }

  /*
   * GPRS functions
   */
  bool gprsConnect(const char* apn, const char* user, const char* pwd) {
    gprsDisconnect();

    sendAT(GF("+SAPBR=3,1,\"Contype\",\"GPRS\""));
    waitResponse();

    sendAT(GF("+SAPBR=3,1,\"APN\",\""), apn, '"');
    waitResponse();

    if (user && strlen(user) > 0) {
      sendAT(GF("+SAPBR=3,1,\"USER\",\""), user, '"');
      waitResponse();
    }
    if (pwd && strlen(pwd) > 0) {
      sendAT(GF("+SAPBR=3,1,\"PWD\",\""), pwd, '"');
      waitResponse();
    }

    sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');
    waitResponse();

    sendAT(GF("+CGACT=1,1"));
    waitResponse(60000L);

    // Open a GPRS context
    sendAT(GF("+SAPBR=1,1"));
    waitResponse(85000L);
    // Query the GPRS context
    sendAT(GF("+SAPBR=2,1"));
    if (waitResponse(30000L) != 1)
      return false;

    sendAT(GF("+CGATT=1"));
    if (waitResponse(60000L) != 1)
      return false;

    // TODO: wait AT+CGATT?

    sendAT(GF("+CIPMUX=1"));
    if (waitResponse() != 1) {
      return false;
    }

    sendAT(GF("+CIPQSEND=1"));
    if (waitResponse() != 1) {
      return false;
    }

    sendAT(GF("+CIPRXGET=1"));
    if (waitResponse() != 1) {
      return false;
    }

    sendAT(GF("+CSTT=\""), apn, GF("\",\""), user, GF("\",\""), pwd, GF("\""));
    if (waitResponse(60000L) != 1) {
      return false;
    }

    sendAT(GF("+CIICR"));
    if (waitResponse(60000L) != 1) {
      return false;
    }

    sendAT(GF("+CIFSR;E0"));
    if (waitResponse(10000L) != 1) {
      return false;
    }

    sendAT(GF("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\""));
    if (waitResponse() != 1) {
      return false;
    }

    return true;
  }

  bool gprsDisconnect() {
    sendAT(GF("+CIPSHUT"));
    if (waitResponse(60000L) != 1)
      return false;

    sendAT(GF("+CGATT=0"));
    if (waitResponse(60000L) != 1)
      return false;

    return true;
  }

  bool isGprsConnected() {
    sendAT(GF("+CGATT?"));
    if (waitResponse(GF(GSM_NL "+CGATT:")) != 1) {
      return false;
    }
    int res = stream.readStringUntil('\n').toInt();
    waitResponse();
    if (res != 1)
      return false;

    sendAT(GF("+CIFSR;E0")); // Another option is to use AT+CGPADDR=1
    if (waitResponse() != 1)
      return false;

    return true;
  }

  String getLocalIP() {
    sendAT(GF("+CIFSR;E0"));
    String res;
    if (waitResponse(10000L, res) != 1) {
      return "";
    }
    res.replace(GSM_NL "OK" GSM_NL, "");
    res.replace(GSM_NL, "");
    res.trim();
    return res;
  }

  IPAddress localIP() {
    return TinyGsmIpFromString(getLocalIP());
  }

  /*
   * Phone Call functions
   */

  bool setGsmBusy(bool busy = true) {
    sendAT(GF("+GSMBUSY="), busy ? 1 : 0);
    return waitResponse() == 1;
  }

  bool callAnswer() {
    sendAT(GF("A"));
    return waitResponse() == 1;
  }

  // Returns true on pick-up, false on error/busy
  bool callNumber(const String& number) {
    if (number == GF("last")) {
      sendAT(GF("DL"));
    } else {
      sendAT(GF("D"), number, ";");
    }
    int status = waitResponse(60000L,
                              GFP(GSM_OK),
                              GF("BUSY" GSM_NL),
                              GF("NO ANSWER" GSM_NL),
                              GF("NO CARRIER" GSM_NL));
    switch (status) {
    case 1:  return true;
    case 2:
    case 3:  return false;
    default: return false;
    }
  }

  bool callHangup() {
    sendAT(GF("H"));
    return waitResponse() == 1;
  }

  // 0-9,*,#,A,B,C,D
  bool dtmfSend(char cmd, int duration_ms = 100) {
    duration_ms = constrain(duration_ms, 100, 1000);

    sendAT(GF("+VTD="), duration_ms / 100); // VTD accepts in 1/10 of a second
    waitResponse();

    sendAT(GF("+VTS="), cmd);
    return waitResponse(10000L) == 1;
  }

  /*
   * Messaging functions
   */

  String sendUSSD(const String& code) {
    sendAT(GF("+CSCS=\"HEX\""));
    waitResponse();
    sendAT(GF("+CUSD=1,\""), code, GF("\""));
    if (waitResponse() != 1) {
      return "";
    }
    if (waitResponse(10000L, GF(GSM_NL "+CUSD:")) != 1) {
      return "";
    }
    stream.readStringUntil('"');
    String hex = stream.readStringUntil('"');
    stream.readStringUntil(',');
    int dcs = stream.readStringUntil('\n').toInt();

    if (dcs == 15) {
      return TinyGsmDecodeHex8bit(hex);
    } else if (dcs == 72) {
      return TinyGsmDecodeHex16bit(hex);
    } else {
      return hex;
    }
  }

  int getSMSInterrupt() {
    sendAT(GF("+CFGRI?"));
    if (waitResponse(GF(GSM_NL "+CFGRI:")) != 1) return -1;
    return stream.readStringUntil('\n').toInt();
  }

  bool setSMSInterrupt(int status) {
    sendAT(GF("+CFGRI="), status);
    if (waitResponse() != 1) return false;
    return true;
  }

  int countSMS() {
    sendAT(GF("+CPMS?"));
    if (waitResponse(GF(GSM_NL "+CPMS:")) != 1) return -1;
    streamSkipUntil(',');
    int count = stream.readStringUntil(',').toInt();
    waitResponse();
    return count;
  }

  bool deleteSMS() {
    sendAT(GF("+CMGDA=\"DEL ALL\""));
    return waitResponse() == 1;
  }

  bool deleteSMS(int i) {
    sendAT(GF("+CMGD="), i);
    return waitResponse() == 1;
  }

//  enum {
//    unread = 0,
//    read,
//    unsent,
//    sent,
//  } GsmSmsStat;

  String listSMS(TinyGsmSmsStatusStr status = TinyGsmSmsStatus::all, bool preserveStatus = true) { // GsmSmsStat stat = 0
    sendAT(GF("+CMGL=\""), status, "\",", preserveStatus ? 1 : 0); // not change status
    String result;
    while (waitResponse(GF(GSM_NL "+CMGL:"), GFP(GSM_OK), GFP(GSM_ERROR)) == 1)
      result += (char)stream.readStringUntil(',').toInt();
    return result;
  }

  bool readSMS(int index, String& message, String& sender) {
    sendAT(GF("+CMGR="), i);
    if (waitResponse(GF(GSM_NL "+CMGR:"), GFP(GSM_OK), GFP(GSM_ERROR)) == 1) {
      streamSkipUntil(',');
      streamSkipUntil('"');
      sender = stream.readStringUntil('"');
      streamSkipUntil('\n');
      message = stream.readStringUntil('\n');
      return waitResponse() == 1;
    }
    return false;
  }

  bool sendSMS(const String& number, const String& text) {
    sendAT(GF("+CMGS=\""), number, GF("\""));
    if (waitResponse(GF(">")) != 1) return false;
    stream.print(text);
    stream.write((char)0x1A);
    stream.flush();
    return waitResponse(60000L) == 1;
  }

  bool sendSMS_UTF16(const String& number, const void* text, size_t len) {
    sendAT(GF("+CSCS=\"HEX\""));
    waitResponse();
    sendAT(GF("+CSMP=17,167,0,8"));
    waitResponse();

    sendAT(GF("+CMGS=\""), number, GF("\""));
    if (waitResponse(GF(">")) != 1) {
      return false;
    }

    uint16_t* t = (uint16_t*)text;
    for (size_t i=0; i<len; i++) {
      uint8_t c = t[i] >> 8;
      if (c < 0x10) { stream.print('0'); }
      stream.print(c, HEX);
      c = t[i] & 0xFF;
      if (c < 0x10) { stream.print('0'); }
      stream.print(c, HEX);
    }
    stream.write((char)0x1A);
    stream.flush();
    return waitResponse(60000L) == 1;
  }


  /*
   * Location functions
   */

  String getGsmLocation() {
    sendAT(GF("+CIPGSMLOC=1,1"));
    if (waitResponse(10000L, GF(GSM_NL "+CIPGSMLOC:")) != 1) {
      return "";
    }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  /*
   * Battery functions
   */
  // Use: float vBatt = modem.getBattVoltage() / 1000.0;
  uint16_t getBattVoltage() {
    sendAT(GF("+CBC"));
    if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
      return 0;
    }
    streamSkipUntil(','); // Skip
    streamSkipUntil(','); // Skip

    uint16_t res = stream.readStringUntil(',').toInt();
    waitResponse();
    return res;
  }

  int getBattPercent() {
    sendAT(GF("+CBC"));
    if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
      return false;
    }
    stream.readStringUntil(',');
    int res = stream.readStringUntil(',').toInt();
    waitResponse();
    return res;
  }

protected:

  bool modemConnect(const char* host, uint16_t port, uint8_t mux, bool ssl = false) {
    sendAT(GF("+CIPSSL="), ssl);
    int rsp = waitResponse();
    if (ssl && rsp != 1) {
      return false;
    }
    sendAT(GF("+CIPSTART="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","), port);
    rsp = waitResponse(75000L,
                       GF("CONNECT OK" GSM_NL),
                       GF("CONNECT FAIL" GSM_NL),
                       GF("ALREADY CONNECT" GSM_NL));
    return (1 == rsp);
  }

  int modemSend(const void* buff, size_t len, uint8_t mux) {
    sendAT(GF("+CIPSEND="), mux, ',', len);
    if (waitResponse(GF(">")) != 1) {
      return -1;
    }
    stream.write((uint8_t*)buff, len);
    stream.flush();
    if (waitResponse(GF(GSM_NL "DATA ACCEPT:")) != 1) {
      return -1;
    }
    streamSkipUntil(','); // Skip mux
    return stream.readStringUntil('\n').toInt();
  }

  size_t modemRead(size_t size, uint8_t mux) {
#ifdef TINY_GSM_USE_HEX
    sendAT(GF("+CIPRXGET=3,"), mux, ',', size);
    if (waitResponse(GF("+CIPRXGET:")) != 1) {
      return 0;
    }
#else
    sendAT(GF("+CIPRXGET=2,"), mux, ',', size);
    if (waitResponse(GF("+CIPRXGET:")) != 1) {
      return 0;
    }
#endif
    streamSkipUntil(','); // Skip mode 2/3
    streamSkipUntil(','); // Skip mux
    size_t len = stream.readStringUntil(',').toInt();
    sockets[mux]->sock_available = stream.readStringUntil('\n').toInt();

    for (size_t i=0; i<len; i++) {
#ifdef TINY_GSM_USE_HEX
      while (stream.available() < 2) { TINY_GSM_YIELD(); }
      char buf[4] = { 0, };
      buf[0] = stream.read();
      buf[1] = stream.read();
      char c = strtol(buf, NULL, 16);
#else
      while (!stream.available()) { TINY_GSM_YIELD(); }
      char c = stream.read();
#endif
      sockets[mux]->rx.put(c);
    }
    waitResponse();
    return len;
  }

  size_t modemGetAvailable(uint8_t mux) {
    sendAT(GF("+CIPRXGET=4,"), mux);
    size_t result = 0;
    if (waitResponse(GF("+CIPRXGET:")) == 1) {
      streamSkipUntil(','); // Skip mode 4
      streamSkipUntil(','); // Skip mux
      result = stream.readStringUntil('\n').toInt();
      waitResponse();
    }
    if (!result) {
      sockets[mux]->sock_connected = modemGetConnected(mux);
    }
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    sendAT(GF("+CIPSTATUS="), mux);
    int res = waitResponse(GF(",\"CONNECTED\""), GF(",\"CLOSED\""), GF(",\"CLOSING\""), GF(",\"INITIAL\""));
    waitResponse();
    return 1 == res;
  }

public:

  /* Utilities */

  template<typename T>
  void streamWrite(T last) {
    stream.print(last);
  }

  template<typename T, typename... Args>
  void streamWrite(T head, Args... tail) {
    stream.print(head);
    streamWrite(tail...);
  }

  bool streamSkipUntil(char c) {
    for (unsigned long timeout = millis() + 1000L; 0 < (long)(timeout - millis()); ) {
    if (!stream.available()) {
        TINY_GSM_YIELD();
        continue;
      }
      if (stream.read() == c)
        return true;
    }
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., GSM_NL);
    stream.flush();
    TINY_GSM_YIELD();
    //DBG("### AT:", cmd...);
  }

  uint8_t waitResponse(unsigned long timeout, String& data,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL)
  {
    data.reserve(64);
    uint8_t index = 0;
    for (timeout += millis(); !index && 0 < (long)(timeout - millis()); ) {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        int a = stream.read();
        if (a <= 0) continue; // Skip 0x00 bytes, just in case
        data += (char)a;
        if (r1 && data.endsWith(r1)) index = 1; else
        if (r2 && data.endsWith(r2)) index = 2; else
        if (r3 && data.endsWith(r3)) index = 3; else
        if (r4 && data.endsWith(r4)) index = 4;
    
        if (index) break; else
        if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) {
          String mode = stream.readStringUntil(',');
          if (mode.toInt() == 1) {
            int mux = stream.readStringUntil('\n').toInt();
            if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
              sockets[mux]->got_data = true;
            }
            data = "";
          } else {
            data += mode;
          }
        } else if (data.endsWith(GF("CLOSED" GSM_NL))) {
          int nl = data.lastIndexOf(GSM_NL, data.length()-8);
          int coma = data.indexOf(',', nl+2);
          int mux = data.substring(nl+2, coma).toInt();
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->sock_connected = false;
          }
          data = "";
          DBG("### Closed: ", mux);
        }
      }
    }
    if (!index) DBG("########### TIMEOUT:\"", data, "\"");
    return index;
  }

  uint8_t waitResponse(unsigned long timeout,
                       GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL)
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4);
  }

  uint8_t waitResponse(GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                       GsmConstStr r3=NULL, GsmConstStr r4=NULL)
  {
    return waitResponse(1000, r1, r2, r3, r4);
  }

protected:
  Stream&       stream;
  GsmClient*    sockets[TINY_GSM_MUX_COUNT];
};

#endif
