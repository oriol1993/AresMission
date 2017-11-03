#include <Buffer.h>
#include <Arduino.h>

void Buffer::reset()
{
  head = 0;
  tail = 0;
  for(uint16_t ii=0;ii<buff_sz;ii++){posicio[ii]=0;}
}

void Buffer::CarregarBuffer(byte data[], uint16_t num)
{
  for (uint16_t i = 0; i < num; i++) {
    posicio[head++] = data[i];
    if (head == buff_sz) {
      head = 0;
    }
  }
}

void Buffer::DescarregarBuffer(byte data[],uint16_t num)
{
  for (uint16_t i = 0; i < num; i++) {
    data[i] = posicio[tail++];
    if (tail == buff_sz) {
      tail = 0;
    }
  }
}

bool Buffer::Check(uint16_t available_bytes)
{
  posicionsPlenes = head - tail;
  if(posicionsPlenes<0){
    posicionsPlenes+=buff_sz;
  }
  return posicionsPlenes >= available_bytes;
}

bool Buffer::spaceAvailable(uint16_t numeroBytes)
{
  return !Check(buff_sz-numeroBytes);
}
