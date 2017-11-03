#include <Arduino.h>

#define buff_sz 400
#define page_sz 256

class Buffer
{
  public:
    void reset();
    void CarregarBuffer(byte data[],uint16_t num);
    void DescarregarBuffer(byte data[],uint16_t num);
    bool Check(uint16_t available_bytes);
    bool spaceAvailable(uint16_t numeroBytes);

  private:
    byte posicio[buff_sz];
    uint16_t head = 0;    //Primera posicio lliure
    uint16_t tail = 0;    //Ultima posicio ocupada
    int posicionsPlenes;
};
