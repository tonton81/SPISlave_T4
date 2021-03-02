#if !defined(_SPISlave_T4_H_)
#define _SPISlave_T4_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include <SPI.h>

typedef enum SPI_BITS {
  SPI_8_BITS = 8,
  SPI_16_BITS = 16,
  SPI_32_BITS = 32,
} SPI_BITS;

typedef void (*_SPI_ptr)();

#define SPISlave_T4_CLASS template<SPIClass* port = nullptr, SPI_BITS bits = SPI_8_BITS>
#define SPISlave_T4_FUNC template<SPIClass* port, SPI_BITS bits>
#define SPISlave_T4_OPT SPISlave_T4<port, bits>

extern SPIClass SPI;

class SPISlave_T4_Base {
  public:
    virtual void SLAVE_ISR();
};

//static SPISlave_T4_Base* _LPSPI1 = nullptr;
//static SPISlave_T4_Base* _LPSPI2 = nullptr;
//static SPISlave_T4_Base* _LPSPI3 = nullptr;
static SPISlave_T4_Base* _LPSPI4 = nullptr;

SPISlave_T4_CLASS class SPISlave_T4 : public SPISlave_T4_Base {
  public:
    SPISlave_T4();
    void begin();
    uint32_t transmitErrors();
    void onReceive(_SPI_ptr handler) { _spihandler = handler; }
    bool active();
    bool available();
    void sniffer(bool enable = 1);
    void swapPins(bool enable = 1);
    void pushr(uint32_t data);
    uint32_t popr();

  private:
    _SPI_ptr _spihandler = nullptr;
    void SLAVE_ISR();
    int _portnum = 0;
    uint32_t nvic_irq = 0;
    uint32_t transmit_errors = 0;
    bool sniffer_enabled = 0;
};

#include "SPISlave_T4.tpp"
#endif