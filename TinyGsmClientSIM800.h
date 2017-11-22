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

#define TIMEOUT_SHORTEST (TIMEOUT_SHORTER>>2)
#define TIMEOUT_SHORTER (TIMEOUT_SHORT>>2)
#define TIMEOUT_SHORT (TIMEOUT_NORMAL>>2)
#define TIMEOUT_NORMAL 2000UL
#define TIMEOUT_LONG (TIMEOUT_NORMAL<<2)
#define TIMEOUT_LONGER (TIMEOUT_LONG<<2)
#define TIMEOUT_LONGEST (TIMEOUT_LONGER<<2)

#define GSM_NL "\r\n"

#define AT_OK       0
#define AT_ERROR   -1
#define AT_TIMEOUT -2

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
        timeout = millis() + TIMEOUT_SHORT;
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
    return 1 == read(&c, 1) ? c : -1;
  }

  virtual int peek() { return -1; } //TODO
  virtual void flush() { at->stream.flush(); }

  virtual uint8_t connected() {
    return available() ? true : sock_connected;
  }
  virtual operator bool() {
    return connected();
  }
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
    if (!testAT()) return false;
    //delay(TIMEOUT_SHORT);

    sendAT(GF("&FZ"));  // Factory + Reset
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("E0"));   // Echo Off
    if (AT_OK != waitResponse()) return false;
    
    // PREFERRED SMS STORAGE
    sendAT(GF("+CPMS="), GF("\"SM\""), GF(","), GF("\"SM\""), GF(","), GF("\"SM\""));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    sendAT(GF("+CMGF=1")); // SMS set TEXT mode
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("+CSDH=0")); // SMS don't show all header values
    if (AT_OK != waitResponse()) return false;

    getSimStatus();
    return true;
  }

  void setBaud(unsigned long baud) {
    sendAT(GF("+IPR="), baud);
  }

  bool testAT(unsigned long timeout = 10000L) {
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      streamWrite(GF("AAAAA"));  // extra A's to help detect the baud rate
      sendAT(GF(""));
      if (AT_OK == waitResponse(TIMEOUT_SHORT)) return true;
    }
    return false;
  }

  void handleSockets() {
    for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
      GsmClient* sock = sockets[mux];
      if (sock && sock->got_data) {
        sock->got_data = false;
        sock->sock_available = modemGetAvailable(mux);
      }
    }
  }

  void handleUnsolicited(String &data) {
    // AAAAAT
    // +CFUN: 1    // +CPIN: READY
    if (data.endsWith(GF(GSM_NL "+CMTI:"))) handleCMTI(data);
    else if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) handleCIPRXGET(data);
    else if (data.endsWith(GF("CLOSED" GSM_NL))) handleCLOSE(data);
  }

  void maintain() {
    handleSockets();
//    while (stream.available()) waitResponse(10); // TODO: dont wait OK/ERROR

    if (!stream.available()) return;
    String data;
    data.reserve(16);
    do {
      data += (char)stream.read();
      handleUnsolicited(data);
      if (!stream.available()) delay(1); // TINY_GSM_YIELD();
    } while (stream.available());
  }

  bool factoryDefault() {
    sendAT(GF("&FZE0&W"));  // Factory + Reset + Echo Off + Write
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("+IPR=0"));   // Auto-baud
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("+IFC=0,0")); // No Flow Control
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("+ICF=3,3")); // 8 data 0 parity 1 stop
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("+CSCLK=0")); // Disable Slow Clock
    if (AT_OK != waitResponse()) return false;
    
    sendAT(GF("&W"));       // Write configuration
    if (AT_OK != waitResponse()) return false;
  }

  bool getModemInfo(String &s) {
    sendAT(GF("I"));
    if (!streamSkipUntil(GSM_NL[1])) return false;
    s = stream.readStringUntil(GSM_NL[0]);
    return AT_OK == waitResponse();
  }

  bool hasSSL() {
    sendAT(GF("+CIPSSL=?"));
    if (1 != waitResponse(GF(GSM_NL "+CIPSSL:"))) return false;
    return AT_OK == waitResponse();
  }

  /*
   * Power functions
   */

  bool restart() {
    if (!testAT()) return false;
    
    sendAT(GF("+CFUN=0"));
    if (AT_OK != waitResponse(TIMEOUT_LONG)) return false;
    
    sendAT(GF("+CFUN=1,1"));
    if (AT_OK != waitResponse(TIMEOUT_LONG)) return false;
    
    delay(TIMEOUT_NORMAL);
    return init();
  }

  bool poweroff() {
    sendAT(GF("+CPOWD=1"));
    return 1 == waitResponse(GF("NORMAL POWER DOWN"));
  }

  bool radioOff() {
    sendAT(GF("+CFUN=0"));
    if (AT_OK != waitResponse(TIMEOUT_LONG)) return false;
    delay(3000);
    return true;
  }

  /*
    During sleep, the SIM800 module has its serial communication disabled. In order to reestablish communication
    pull the DRT-pin of the SIM800 module LOW for at least 50ms. Then use this function to disable sleep mode.
    The DTR-pin can then be released again.
  */
  bool sleepEnable(bool enable = true) {
    sendAT(GF("+CSCLK="), enable ? 1 : 0);
    return AT_OK == waitResponse();
  }

  /*
   * SIM card functions
   */

  bool simUnlock(const char *pin) {
    sendAT(GF("+CPIN=\""), pin, GF("\""));
    return AT_OK == waitResponse();
  }

  bool getSimCCID(String &s) {
    sendAT(GF("+ICCID"));
    if (1 != waitResponse(GF(GSM_NL "+ICCID:"))) return false;
    s = stream.readStringUntil(GSM_NL[0]);
    return AT_OK == waitResponse();
  }

  bool getIMEI(String &s) {
    sendAT(GF("+GSN"));
    if (!streamSkipUntil(GSM_NL[1])) return false;
    s = stream.readStringUntil(GSM_NL[0]);
    return AT_OK == waitResponse();
  }

  SimStatus getSimStatus(unsigned long timeout = 10000L) {
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      sendAT(GF("+CPIN?"));
      if (1 != waitResponse(GF(GSM_NL "+CPIN:"))) {
        delay(1000);
        continue;
      }
      int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK")); //GF("NOT INSERTED")
      if (0 < status) waitResponse();
      switch (status) {
      case 1: return SIM_READY;
      case 2:
      case 3: return SIM_LOCKED;
      }
      break;
    }
    return SIM_ERROR;
  }

  RegStatus getRegistrationStatus() {
    sendAT(GF("+CREG?"));
    if (1 != waitResponse(GF(GSM_NL "+CREG:"))) return REG_UNKNOWN;
    streamSkipUntil(','); // Skip format (0)
    int status = stream.readStringUntil(GSM_NL[1]).toInt();
    waitResponse();
    return (RegStatus)status;
  }

  bool getOperator(String &s) {
    sendAT(GF("+COPS?"));
    if (1 != waitResponse(GF(GSM_NL "+COPS:"))) return false;
    if (!streamSkipUntil('"')) return false;
    s = stream.readStringUntil('"');
    return AT_OK == waitResponse();
  }

  /*
   * Generic network functions
   */
  bool getSignalQuality(int &i) {
    sendAT(GF("+CSQ"));
    if (1 != waitResponse(GF(GSM_NL "+CSQ:"))) return false;
    i = stream.readStringUntil(',').toInt();
    return AT_OK == waitResponse();
  }

  bool isNetworkConnected() {
    RegStatus s = getRegistrationStatus();
    return s == REG_OK_HOME || s == REG_OK_ROAMING;
  }

  bool waitForNetwork(unsigned long timeout = 60000L) {
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      if (isNetworkConnected()) return true;
      delay(TIMEOUT_SHORT);
    }
    return false;
  }

  /*
   * GPRS functions
   */
  bool gprsConnect(const char *apn, const char *user, const char *password) {
    gprsDisconnect();

    sendAT(GF("+SAPBR=3,1,\"Contype\",\"GPRS\""));
    if (AT_OK != waitResponse()) return false;

    sendAT(GF("+SAPBR=3,1,\"APN\",\""), apn, '"');
    if (AT_OK != waitResponse()) return false;

    if (user && strlen(user) > 0) {
      sendAT(GF("+SAPBR=3,1,\"USER\",\""), user, '"');
      if (AT_OK != waitResponse()) return false;
    }
    if (password && strlen(password) > 0) {
      sendAT(GF("+SAPBR=3,1,\"PWD\",\""), password, '"');
      if (AT_OK != waitResponse()) return false;
    }

    sendAT(GF("+CGDCONT=1,\"IP\",\""), apn, '"');
    if (AT_OK != waitResponse()) return false;

    sendAT(GF("+CGACT=1,1"));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    // Open a GPRS context
    sendAT(GF("+SAPBR=1,1"));
    if (AT_OK != waitResponse(TIMEOUT_LONGEST)) return false;
    
  // Query the GPRS context
    sendAT(GF("+SAPBR=2,1"));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    sendAT(GF("+CGATT=1"));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    // TODO: wait AT+CGATT?

    sendAT(GF("+CIPMUX=1"));
    if (AT_OK != waitResponse()) return false;

    sendAT(GF("+CIPQSEND=1"));
    if (AT_OK != waitResponse()) return false;

    sendAT(GF("+CIPRXGET=1"));
    if (AT_OK != waitResponse()) return false;

    sendAT(GF("+CSTT=\""), apn, GF("\",\""), user, GF("\",\""), password, GF("\""));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    sendAT(GF("+CIICR"));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    sendAT(GF("+CIFSR;E0"));
    if (AT_OK != waitResponse(TIMEOUT_LONG)) return false;

    sendAT(GF("+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\""));
    if (AT_OK != waitResponse()) return false;

    return true;
  }

  bool gprsDisconnect() {
    sendAT(GF("+CIPSHUT"));
    if (1 != waitResponse(GF(GSM_NL "SHUT OK" GSM_NL), TIMEOUT_LONGER)) return false;

    sendAT(GF("+CGATT=0"));
    if (AT_OK != waitResponse(TIMEOUT_LONGER)) return false;

    return true;
  }

  bool isGprsConnected() {
    sendAT(GF("+CGATT?"));
    
    if (1 != waitResponse(GF(GSM_NL "+CGATT:"))) return false;
    int res = stream.readStringUntil(GSM_NL[1]).toInt();
    waitResponse();
    if (1 != res) return false;

    sendAT(GF("+CIFSR;E0")); // Another option is to use AT+CGPADDR=1
    if (AT_OK != waitResponse(TIMEOUT_LONG)) return false;

    return true;
  }

  bool getLocalIP(String &s) {
    sendAT(GF("+CIFSR;E0"));
    if (!streamSkipUntil(GSM_NL[1], TIMEOUT_LONG)) return false;
    s = stream.readStringUntil(GSM_NL[0]);
    return AT_OK == waitResponse();
  }

  IPAddress localIP() {
    String ip;
    return getLocalIP(ip) ? TinyGsmIpFromString(ip) : IPAddress();
  }

  /*
   * Phone Call functions
   */

  bool setGsmBusy(bool busy = true) {
    sendAT(GF("+GSMBUSY="), busy ? 1 : 0);
    return AT_OK == waitResponse();
  }

  bool callAnswer() {
    sendAT(GF("A"));
    return AT_OK == waitResponse();
  }

  // Returns true on pick-up, false on error/busy
  bool callNumber(const String& number) {
    if (number == GF("last")) sendAT(GF("DL"));
    else sendAT(GF("D"), number, ";");
    return AT_OK == waitResponse(TIMEOUT_LONGER,
                              //GFP(GSM_OK),
                              GF("BUSY" GSM_NL),
                              GF("NO ANSWER" GSM_NL),
                              GF("NO CARRIER" GSM_NL));
//    switch (status) {
//    case 1:  return true;
//    case 2:
//    case 3:  return false;
//    default: return false;
//    }
  }

  bool callHangup() {
    sendAT(GF("H"));
    return AT_OK == waitResponse();
  }

  // 0-9,*,#,A,B,C,D
  bool dtmfSend(char cmd, int duration_ms = 100) {
    duration_ms = constrain(duration_ms, 100, 1000);

    sendAT(GF("+VTD="), duration_ms / 100); // VTD accepts in 1/10 of a second
    waitResponse();

    sendAT(GF("+VTS="), cmd);
    return AT_OK == waitResponse(TIMEOUT_LONG);
  }

  /*
   * Messaging functions
   */

/*  String sendUSSD(const String& code) {
    sendAT(GF("+CSCS=\"HEX\""));
    if (AT_OK != waitResponse()) return "";
    sendAT(GF("+CUSD=1,\""), code, GF("\""));
    if (1 != waitResponse(TIMEOUT_LONG, GF(GSM_NL "+CUSD:"))) return "";
    stream.readStringUntil('"');
    String hex = stream.readStringUntil('"');
    stream.readStringUntil(',');
    switch (stream.readStringUntil(GSM_NL[1]).toInt()) {
    case 15: return TinyGsmDecodeHex8bit(hex);
    case 72: return TinyGsmDecodeHex16bit(hex);
    }
    return hex;
  }*/

/*  int getSMSInterrupt() {
    sendAT(GF("+CFGRI?"));
    if (1 != waitResponse(GF(GSM_NL "+CFGRI:"))) return -1;
    int result = stream.readStringUntil(GSM_NL[1]).toInt();
    waitResponse();
    return result;
  }

  bool setSMSInterrupt(int status) {
    sendAT(GF("+CFGRI="), status);
    return AT_OK == waitResponse();
  }*/

//  bool getSMSNotification() {
//    sendAT(GF("+CNMI?"));
//    return AT_OK == waitResponse();
//  }

#define kSmsStatusReceivedUnread  "REC UNREAD"
#define kSmsStatusReceivedRead    "REC READ"
#define kSmsStatusStoredUnsent    "STO UNSENT"
#define kSmsStatusStoredSent      "STO SENT"
#define kSmsStatusAll             "ALL"

  String smsIndex;
  char getSms() {
    if (0 == smsIndex.length()) return 0;
    char c = smsIndex[0];
    smsIndex.remove(0, 1);
    return c;
  }

  bool setSmsNotification(bool enable) {
    sendAT(GF("+CNMI=2,"), enable ? 1 : 0, GF(",0,0,0")); // "+CNMI=1,"
    bool success = AT_OK == waitResponse();
    if (enable) smsIndex = listSms(kSmsStatusReceivedUnread);
    return success;
  }

  int countSms() {
    sendAT(GF("+CPMS?"));
    if (1 != waitResponse(GF(GSM_NL "+CPMS:"))) return -1;
    streamSkipUntil(',');
    int result = stream.readStringUntil(',').toInt();
    waitResponse();
    return result;
  }

  bool deleteSms(int index) {
    sendAT(GF("+CMGD="), index);
    return AT_OK == waitResponse();
  }

  String listSms(const char *status = kSmsStatusAll, bool preserveStatus = true) { // GsmSmsStat stat = 0
    sendAT(GF("+CMGL=\""), status, "\",", preserveStatus ? 1 : 0); // not change status
    String result;
    while (1 == waitResponse(GF(GSM_NL "+CMGL:")))
      result += (char)stream.readStringUntil(',').toInt();
    return result;
  }

  bool readSms(int index, String& sender, String& message, bool remove = false) {
    sendAT(GF("+CMGR="), index);
    if (1 != waitResponse(GF(GSM_NL "+CMGR:"))) return false;
    streamSkipUntil(',');
    streamSkipUntil('"');
    sender = stream.readStringUntil('"');
    streamSkipUntil(GSM_NL[1]);
    message = stream.readStringUntil(GSM_NL[0]);
    return AT_OK == waitResponse() && (!remove || deleteSms(index));
  }

  bool sendSms(const String &recipient, const String &message) {
    sendAT(GF("+CMGS=\""), recipient, GF("\""));
    if (!streamSkipUntil('>')) return false;
    stream.print(message);
    stream.write(0x1A);
    stream.flush();
    return AT_OK == waitResponse(TIMEOUT_LONGER);
  }

/*  bool sendSms_UTF16(const String& recipient, const uint16_t* message, size_t length) {
    sendAT(GF("+CSCS=\"HEX\""));
    waitResponse();
    sendAT(GF("+CSMP=17,167,0,8"));
    waitResponse();

    sendAT(GF("+CMGS=\""), recipient, GF("\""));
    if (1 != waitResponse(GF(">"))) return false;

    for (size_t i=0; i<length; i++) {
      uint8_t c = message[i] >> 8;
      if (c < 0x10) { stream.print('0'); }
      stream.print(c, HEX);
      c = message[i] & 0xFF;
      if (c < 0x10) { stream.print('0'); }
      stream.print(c, HEX);
    }
    stream.write(0x1A);
    stream.flush();
    return AT_OK == waitResponse(TIMEOUT_LONGER);
  }*/


  /*
   * Location functions
   */

  bool getGsmLocation(String &s) {
    sendAT(GF("+CIPGSMLOC=1,1"));
    if (1 != waitResponse(TIMEOUT_LONG, GF(GSM_NL "+CIPGSMLOC:"))) return false;
    s = stream.readStringUntil(GSM_NL[0]);
    return AT_OK == waitResponse();
  }

  bool getBatteryInfo(int &p, float &v) {
    sendAT(GF("+CBC"));
    if (1 != waitResponse(GF(GSM_NL "+CBC:"))) return false;
    if (!streamSkipUntil(',')) return false;
    p = stream.readStringUntil(',').toInt();
    v = stream.readStringUntil(GSM_NL[0]).toInt() / 1000.0;
    return AT_OK == waitResponse();
  }

protected:
  
  bool modemConnect(const char* host, uint16_t port, uint8_t mux, bool ssl = false) {
    // TODO: test!
    sendAT(GF("+CIPSSL="), ssl);
    if (AT_OK != waitResponse()) return false;

    sendAT(GF("+CIPSTART="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","), port);
    return 1 == waitResponse(TIMEOUT_LONGER,
                       GF("CONNECT OK" GSM_NL),
                       GF("CONNECT FAIL" GSM_NL),
                       GF("ALREADY CONNECT" GSM_NL));
  }

  int modemSend(const void* buff, size_t len, uint8_t mux) {
    sendAT(GF("+CIPSEND="), mux, ',', len);
    if (1 != waitResponse(GF(">"))) return -1;
    stream.write((uint8_t*)buff, len);
    stream.flush();
    if (1 != waitResponse(GF(GSM_NL "DATA ACCEPT:"))) return -1;
    streamSkipUntil(','); // Skip mux
    return stream.readStringUntil(GSM_NL[1]).toInt();
  }

  size_t modemRead(size_t size, uint8_t mux) {
#ifdef TINY_GSM_USE_HEX
    sendAT(GF("+CIPRXGET=3,"), mux, ',', size);
    if (1 != waitResponse(GF("+CIPRXGET:"))) return 0;
#else
    sendAT(GF("+CIPRXGET=2,"), mux, ',', size);
    if (1 != waitResponse(GF("+CIPRXGET:"))) return 0;
#endif
    streamSkipUntil(','); // Skip mode 2/3
    streamSkipUntil(','); // Skip mux
    size_t len = stream.readStringUntil(',').toInt();
    sockets[mux]->sock_available = stream.readStringUntil(GSM_NL[1]).toInt();

    for (size_t i=0; i<len; i++) {
#ifdef TINY_GSM_USE_HEX
      while (stream.available() < 2) { TINY_GSM_YIELD(); }
      char buf[4] = { 0, };
      buf[0] = stream.read();
      buf[1] = stream.read();
      char c = strtol(buf, NULL, 16);
#else
      while (!stream.available()) TINY_GSM_YIELD();
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
    if (1 == waitResponse(GF("+CIPRXGET:"))) {
      streamSkipUntil(','); // Skip mode 4
      streamSkipUntil(','); // Skip mux
      result = stream.readStringUntil(GSM_NL[1]).toInt();
      waitResponse();
    }
    if (!result) sockets[mux]->sock_connected = modemGetConnected(mux);
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    sendAT(GF("+CIPSTATUS="), mux);
    if (1 != waitResponse(GF(",\"CONNECTED\""))) return false;
    waitResponse();
    return true;
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

  bool streamSkipUntil(char c, unsigned long timeout = TIMEOUT_NORMAL) {
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      if (!stream.available()) delay(1);
      else if (c == stream.read()) return true;
    }
    return false;
  }

/*  bool streamReadUntil(char c, String &s, unsigned long timeout = TIMEOUT_NORMAL) {
    s = "";
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      if (!stream.available()) {
        delay(1);
        continue;
      }
      char r = stream.read();
      if (c == r) return true;
      else s += r;
    }
    return false;
  }*/
  void flush() {
    bool flushed = false;
    while (stream.available()) {
      stream.read();
      if (!stream.available()) delay(1);
    }
    if (flushed) DBG("*** FLUSHED");
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    flush();
    streamWrite("AT", cmd..., GSM_NL);
    stream.flush();
    TINY_GSM_YIELD();
    //DBG("### AT:", cmd...);
  }

  bool handleCMTI(String& data) {
    String storage = stream.readStringUntil(',');
    //if (" \"SM\"," != storage) return false;
    smsIndex += (char)stream.readStringUntil(GSM_NL[1]).toInt();
    data = "";
  }

  void handleCIPRXGET(String& data) {
    String mode = stream.readStringUntil(',');
    if (1 != mode.toInt()) {
      data += mode;
      return;
    }
    int mux = stream.readStringUntil(GSM_NL[1]).toInt();
    if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux])
      sockets[mux]->got_data = true;
    data = "";
  }

  void handleCLOSE(String& data) {
    int nl = data.lastIndexOf(GSM_NL, data.length() - 8);
    int coma = data.indexOf(',', nl + 2);
    int mux = data.substring(nl + 2, coma).toInt();
    if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux])
      sockets[mux]->sock_connected = false;
    data = "";
    DBG("### Closed: ", mux);
  }

  int waitResponse(unsigned long timeout, String &data, GsmConstStr r1 = NULL, GsmConstStr r2 = NULL, GsmConstStr r3 = NULL) {
    data.reserve(16);
    for (timeout += millis(); 0 < (long)(timeout - millis()); ) {
      delay(1);
      while (0 < stream.available()) {
//        char c = stream.read();
//        if (c <= 0) continue; // Skip 0x00 bytes, just in case
//        data += c;
        data += (char)stream.read();
        if (r1 && data.endsWith(r1)) return 1;
        if (r2 && data.endsWith(r2)) return 2;
        if (r3 && data.endsWith(r3)) return 3;
        if (data.endsWith(GF("\nOK" GSM_NL))) return AT_OK;
        if (data.endsWith(GF("\nERROR" GSM_NL))) return AT_ERROR;
        handleUnsolicited(data);
      }
    }
    DBG("*** TIMEOUT:\"", data, "\"");
    return AT_TIMEOUT;
  }

  inline int waitResponse(unsigned long timeout, GsmConstStr r1 = NULL, GsmConstStr r2 = NULL, GsmConstStr r3 = NULL) {
	  //DBG("waitResponse ", timeout, r1, r2, r3);
    String data;
    return waitResponse(timeout, data, r1, r2, r3);
  }
  inline int waitResponse(GsmConstStr r1 = NULL, GsmConstStr r2 = NULL, GsmConstStr r3 = NULL) {
    return waitResponse(TIMEOUT_NORMAL, r1, r2, r3);
  }
  //  template<typename... Args>
  //  uint8_t waitResponse(unsigned long timeout, Args... args) {
  //    String data;
  //    return waitResponse(timeout, data, args...);
  //  }

  //  template<typename... Args>
  //  uint8_t waitResponse(Args... args) {
  //    return waitResponse(1000, args...);
  //  }


protected:
  Stream&       stream;
  GsmClient*    sockets[TINY_GSM_MUX_COUNT];
};

#endif
