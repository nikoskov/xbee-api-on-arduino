#include <string.h>

#include "HardwareSerial.h"
#include "WConstants.h"

#include "xbee.h"
#include "xbee_protocol.h"
#include "XBeeReader.h"

#ifndef MIN
# define MIN(a,b) ((a)<(b)?(a):(b))
#endif


XBeeDataFrame& XBeeDataFrame::operator=(const XBeeDataFrame& orig)
{
    memcpy(this->packet, orig.packet, sizeof(orig.packet));
    return *this;
}


int XBeeDataFrame::getApiID()
{
    return ((xbee_pkt_t *)packet)->type;
}


int XBeeDataFrame::getAddress16(unsigned char addr[2])
{
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;


    if (pkt->type == RX16) {
        memcpy(addr, ((xbee_a16_rx_pkt_t *)pkt)->src, 2);
    } else if (pkt->type == IO16) {
        memcpy(addr, ((xbee_io_a16_rx_pkt_t *)pkt)->src, 2);
    } else {
        return -1;
    }
    
    return 0;
}


int XBeeDataFrame::getAddress64(unsigned char addr[8])
{
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;


    if (pkt->type == RX64) {
        memcpy(addr, ((xbee_a64_rx_pkt_t *)pkt)->src, 8);
    } else if (pkt->type == IO64) {
        memcpy(addr, ((xbee_io_a64_rx_pkt_t *)pkt)->src, 8);
    } else {
        return -1;
    }
    
    return 0;
}


int XBeeDataFrame::getNextDataByte()
{
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;
    int pkt_len = ntohs(pkt->hdr.len);


    pkt_len = MIN(pkt_len, XBEE_MAX_DATA_LEN);

    if (pkt->type == RX16) {
        xbee_a16_rx_pkt_t *rx16pkt = (xbee_a16_rx_pkt_t *)packet;
        
        if (dataIndex_ < (ntohs(pkt->hdr.len) - sizeof(xbee_a16_rx_pkt_t) - 1)) {
            return rx16pkt->data[dataIndex_++];
        }
    } else if (pkt->type == RX64) {
        xbee_a64_rx_pkt_t *rx64pkt = (xbee_a64_rx_pkt_t *)packet;
        
        if (dataIndex_ < (ntohs(pkt->hdr.len) - sizeof(xbee_a64_rx_pkt_t) - 1)) {
            return rx64pkt->data[dataIndex_++];
        }
    }
    
    return -1;
}


int XBeeDataFrame::getRSSI()
{
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;


    if (pkt->type == RX16) {
        return ((xbee_a16_rx_pkt_t *)pkt)->rssi;
    } else if (pkt->type == RX64) {
        return ((xbee_a64_rx_pkt_t *)pkt)->rssi;
    } else if (pkt->type == IO16) {
        return ((xbee_io_a16_rx_pkt_t *)pkt)->rssi;
    } else if (pkt->type == IO64) {
        return ((xbee_io_a64_rx_pkt_t *)pkt)->rssi;
    }
    
    return -1;
}


static inline uint16_t sample_len_from_mask(uint16_t mask)
{
    uint8_t len = 0;
    
    
    // Digital; all fit in 1 sample
    if (mask & 0x1ff) len++;
    
    // Analog lines 0-4
    if (len & 0x0200) len++;
    if (len & 0x0400) len++;
    if (len & 0x0800) len++;
    if (len & 0x1000) len++;
    if (len & 0x2000) len++;
    
    return len;
}


// Note: no checking for case where packet is corrupt (says has more samples than
//  really has) if request for higher sample than really contains.
int XBeeDataFrame::getDigital(int array[9], unsigned int index)
{
    uint16_t mask = 0;
    uint16_t pin_vals = 0;
    uint8_t i;
    uint8_t samp_len;
    uint8_t npins = 0;
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;
    xbee_io_a16_rx_pkt_t *io16pkt = (xbee_io_a16_rx_pkt_t *)packet;
    xbee_io_a64_rx_pkt_t *io64pkt = (xbee_io_a64_rx_pkt_t *)packet;
    
    
    for (i = 0; i < 9; i++) {
        array[i] = -1;
    }
    
    if (pkt->type == IO16) {
        mask = ntohs(io16pkt->ch_ind);
        samp_len = sample_len_from_mask(mask);
            
        if ((mask & 0x1ff) != 0 && (index < io16pkt->num_samples)) {
            pin_vals = ntohs(io16pkt->data[index * samp_len]);
        }
    } else if (pkt->type == IO64) {
        mask = ntohs(io64pkt->ch_ind);
        samp_len = sample_len_from_mask(mask);

        if ((mask & 0x1ff) != 0 && (index < io64pkt->num_samples)) {
            pin_vals = ntohs(io64pkt->data[index * samp_len]);
        }
    } else {
        // Unusable packet type
        return -1;
    }
    
    for (i = 0; i < 9; i++) {
        uint16_t b = (1 << i);
        if (mask & b) {
            if (pin_vals & b) {
                array[i] = 1;
            } else {
                array[i] = 0;
            }
            npins++;
        }
    }
    
    return npins;
}


// Note: no checking for case where packet is corrupt (says has more samples than
//  really has) if request for higher sample than really contains.
int XBeeDataFrame::getAnalog(int array[6], unsigned int index)
{
    uint16_t mask = 0;
    uint8_t pos = 0;
    uint8_t npins = 0;
    uint8_t i;
    uint8_t samp_len;
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;
    xbee_io_a16_rx_pkt_t *io16pkt = (xbee_io_a16_rx_pkt_t *)packet;
    xbee_io_a64_rx_pkt_t *io64pkt = (xbee_io_a64_rx_pkt_t *)packet;
    
    
    for (i = 0; i < 6; i++) {
        array[i] = -1;
    }
    
    if (pkt->type == IO16) {
        mask = ntohs(io16pkt->ch_ind);
        samp_len = sample_len_from_mask(mask);
        
        if (mask & 0x1ff) {
            // Skip digital readings
            pos++;
        }
        
        for (i = 0; i < 6; i++) {
            if (mask & (0x200 << i)) {
                array[i] = ntohs(io16pkt->data[index * samp_len + pos]);
                pos++;
                npins++;
            }
        }
    } else if (pkt->type == IO64) {
        mask = ntohs(io64pkt->ch_ind);
        samp_len = sample_len_from_mask(mask);
        
        if (mask & 0x1ff) {
            // Skip digital readings
            pos++;
        }

        for (i = 0; i < 6; i++) {
            if (mask & (0x200 << i)) {
                array[i] = ntohs(io64pkt->data[index * samp_len + pos]);
                pos++;
                npins++;
            }
        }
     } else {
        return -1;
    }

    return npins;
}


int XBeeDataFrame::getTotalSamples()
{
   xbee_pkt_t *pkt = (xbee_pkt_t *)packet;


   if (pkt->type == IO16) {
       return ((xbee_io_a16_rx_pkt_t *)pkt)->num_samples;
   } else if (pkt->type == IO64) {
       return ((xbee_io_a16_rx_pkt_t *)pkt)->num_samples;
   }
   
   return -1;
}


int XBeeDataFrame::getRawPacket(unsigned char *data, int maxlen)
{
    xbee_pkt_t *pkt = (xbee_pkt_t *)packet;
    uint8_t len;
    
    
    len = MIN(maxlen, XBEE_MAX_DATA_LEN);
    len = MIN(len, ntohs(pkt->hdr.len));
    memcpy(data, packet, len);
    
    return len;
}


/* Called from xbee.c to send a packet out to the XBee module */

int xbee_out(xbee_t *xbee_ctx, xbee_pkt_t *pkt, uint8_t len)
{
    XBeeReader *xbee = static_cast<XBeeReader *>(xbee_user_context(*xbee_ctx));
    unsigned char *data = (unsigned char *)pkt;
    int i;
    
    
    for (i = 0; i < len; i++) {
        xbee->serial_.print(data[i]);
    }
    
    
    return 0;
}


int xbee_recv_pkt(xbee_t *xbee_ctx, xbee_pkt_t *pkt, uint8_t len)
{
    XBeeReader *xbee = static_cast<XBeeReader *>(xbee_user_context(*xbee_ctx));
    
    
    xbee->frames_[xbee->buildingFrame_].dataIndex_ = 0;
    xbee->readyFrame_ = xbee->buildingFrame_;
    xbee->buildingFrame_++;
    xbee->buildingFrame_ %= 2;
    xbee->available_ = true;
    
    return 0;
}


void *xbee_alloc_pkt_mem(xbee_t *xbee_ctx, uint8_t direction, uint8_t len)
{
    XBeeReader *xbee = static_cast<XBeeReader *>(xbee_user_context(*xbee_ctx));


    if (direction == XBEE_XMIT) {
        // Not multithreaded (and all data gets dumped at once) so
        //  only need one output buffer.
        return xbee->outData;
    }
    
    if (xbee->readyFrame_ == 0) {
        xbee->buildingFrame_ = 1;
        return xbee->frames_[1].packet;
    } else {
        xbee->buildingFrame_ = 0;
        return xbee->frames_[0].packet;
    }
    
    return NULL;
}


void xbee_free_pkt_mem(xbee_t *xbee_ctx, xbee_pkt_t *pkt)
{
    // memory is all static
}


/* Wait for "OK" from the XBee module */

int XBeeReader::wait_for_ok()
{
    for (int i = 0; i < 2500; i++) {
        int received = 0;
        char buf[3];
            
        while (serial_.available()) {
            buf[received] = serial_.read();
            
            if (buf[received] == 0x0d) {
                buf[received] = '\0';
                if (strcmp("OK", buf)) {
                    return -1;
                }
                    
                return 0;
            }
            
            if (++received >= 3) {
                // Shift buffer over one -- we might have interrupted
                //  data and need to toss out the old stuff...
                buf[0] = buf[1];
                buf[1] = buf[2];
                received--;
            }
        }
        delay(2);
    }

    return -1;
}


int XBeeReader::begin(long speed, bool force_api_mode)
{
    int i;
    
    
    serial_.begin(speed);
    if (force_api_mode) {
        for (i = 0; i < 2; i++) {
            delay(1300);
            serial_.print("+++");
            if (wait_for_ok() < 0) {
                serial_.print("\x0dATCN\x0d");
                continue;
            }
            break;
        }
        if (i == 3)
            return -1;
        serial_.print("ATAP1\x0d");
        if (wait_for_ok() < 0)
            return -1;
    }
    
    xbee_init(&xbee_ctx_);
    xbee_user_context(xbee_ctx_) = this;
    
    return 0;
}


bool XBeeReader::available(void)
{
    return available_;
}


int XBeeReader::send(const char *c, unsigned int len, unsigned int opt,
                 const uint8_t *address, uint8_t addrlen)
{
    if (addrlen == 2) {
        return xbee_send16(&xbee_ctx_, c, len, opt, address);
    } else if (addrlen == 8) {
        return xbee_send64(&xbee_ctx_, c, len, opt, address);
    } else {
        return -1;
    }
}


int XBeeReader::getXBeeReading(XBeeDataFrame &frame)
{
    if (!available_) {
        return -1;
    }
    
    available_ = 0;
    
    frame = frames_[readyFrame_];

    return 0;
}


void XBeeReader::poll()
{
    while (serial_.available()) {
        char c;
        
        c = serial_.read();
        xbee_in(&xbee_ctx_, &c, 1); 
    }
}
