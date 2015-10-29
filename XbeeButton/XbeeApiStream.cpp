#include "XbeeApiStream.h"

XbeeApiStream::XbeeApiStream() {
  xbee = XBee();
  coordAddr64 = XBeeAddress64(0x00000000, 0x0000ffff);
}

void XbeeApiStream::begin(Stream &s) {
  serialStream = &s;
  xbee.begin(s);
}

int XbeeApiStream::available(void) {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable() && xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
    rxResponse = ZBRxResponse();
    xbee.getResponse().getZBRxResponse(rxResponse);
    uint8_t* data = rxResponse.getData();
    for (int i = 0; i < rxResponse.getDataLength(); i++)
      rxQueue.enqueue(data[i]);
  }
  return rxQueue.count();
}

int XbeeApiStream::read(void) {
  if (!rxQueue.isEmpty()) return rxQueue.dequeue();
  else return -1;
}

int XbeeApiStream::readBytesUntil(uint8_t c, uint8_t* buf, size_t len){
  int bytesWritten = 0;
  while(!rxQueue.isEmpty() && bytesWritten < len){
    *buf = rxQueue.dequeue();
    bytesWritten++;
    if(*buf == c) break;
    buf++;
  }
  return bytesWritten;
}

size_t XbeeApiStream::write(const uint8_t* buf, size_t len) {
  if (len == 0) return 0;
  ZBTxRequest txResponse = ZBTxRequest(coordAddr64, (uint8_t*)buf, len);
  xbee.send(txResponse);
  return len;
}

int XbeeApiStream::peek(void) {
  if (!rxQueue.isEmpty())  return rxQueue.peek();
  else return -1;
}

void XbeeApiStream::flush(void) {
  serialStream->flush();
}

