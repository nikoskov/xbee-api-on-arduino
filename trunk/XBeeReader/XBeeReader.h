#ifndef XBEE_READER_H
#define XBEE_READER_H

#include "HardwareSerial.h"
#include "xbee.h"
#include "xbee_protocol.h"


class XBeeDataFrame {
  protected:
    // To save space overall while avoiding dynamic memory allocation
    //  just always keep things around in raw pkt form (rather than also keeping
    //  cooked data)
    uint8_t packet[XBEE_MAX_DATA_LEN];
    uint8_t dataIndex_;
    friend void *xbee_alloc_pkt_mem(xbee_t *xbee_ctx, uint8_t direction, uint8_t len);
    friend int xbee_recv_pkt(xbee_t *xbee_ctx, xbee_pkt_t *pkt, uint8_t len);

  public:
    static const int MODEM_STATUS = 0x8a;
    static const int CMD_RESPONSE = 0x88;
    static const int TX_STATUS = 0x89;
    static const int RX16 = 0x81;
    static const int RX64 = 0x80;
    static const int IO16 = 0x83;
    static const int IO64 = 0x82;
        
    XBeeDataFrame() { dataIndex_ = 0; }
    XBeeDataFrame(const xbee_pkt_t *pkt);
    
    XBeeDataFrame& operator=(const XBeeDataFrame& orig); 
    
    int getAddress16(unsigned char addr[2]);
    int getAddress64(unsigned char addr[8]);
    int getApiID(void);
    int getDigital(int array[9], unsigned int index = 0);
    int getAnalog(int array[6], unsigned int index = 0);
    int getRSSI(void);
    int getTotalSamples(void);
    int getRawPacket(unsigned char *data, int maxlen);
    int getNextDataByte(void);
    
    friend class XBeeReader;
};


class XBeeReader {
  private:
    HardwareSerial serial_;
    xbee_t xbee_ctx_;
    XBeeDataFrame frames_[2];
    bool available_;
    int readyFrame_;
    int buildingFrame_;
    unsigned char outData[XBEE_MAX_DATA_LEN];
    
    XBeeReader();  // Not allowed
    int wait_for_ok(void);
    
    friend int xbee_out(xbee_t *xbee_ctx, xbee_pkt_t *pkt, uint8_t len);
    friend int xbee_recv_pkt(xbee_t *xbee_ctx, xbee_pkt_t *pkt, uint8_t len);
    friend void *xbee_alloc_pkt_mem(xbee_t *xbee_ctx, uint8_t direction, uint8_t len);

  public:
    XBeeReader(HardwareSerial serial) : serial_(serial), readyFrame_(-1) , buildingFrame_(-1) 
    { xbee_init(&xbee_ctx_); xbee_user_context(xbee_ctx_) = this;}
    int begin(long speed = 9600, bool force_api_mode = 1);
    int send(const char *c, unsigned int len, unsigned int opt = 0,
             const uint8_t *address = (const uint8_t *)"\xff\xff", uint8_t addrlen = 2);
    int getXBeeReading(XBeeDataFrame &data);
    void poll(void);
    bool available(void);
};

#endif /* #ifndef XBEE_READER_H */
