#include <Arduino.h>
#include <Xbee.h>
#include <Stream.h>
#include <QueueArray.h>

#ifndef XbeeApiStream_h
#define XbeeApiStream_h

class XbeeApiStream : public Stream {
  private:
    XBee xbee;
    XBeeAddress64 coordAddr64;
    ZBRxResponse rxResponse;
    ZBTxRequest txResponse;
    Stream* serialStream;
    QueueArray <uint8_t> rxQueue;

  public:
    XbeeApiStream();
    void begin(Stream &s);
    size_t write(uint8_t c) {
      return write(&c, 1);
    }
    size_t write(const char* str) {
      if (str == NULL) return 0;
      else return write((const uint8_t*)str, strlen(str));
    }
    size_t write(const char* buf, size_t len) {
      return write((const uint8_t*)buf, len);
    }
    size_t write(String str){
      return write(str.c_str());
    }
    virtual size_t write(const uint8_t* buf, size_t len);
    virtual int read(void);
    virtual int readBytesUntil(uint8_t c, uint8_t* buf, size_t len);
    virtual int peek(void);
    virtual void flush(void);
    virtual int available(void);
};

#endif
