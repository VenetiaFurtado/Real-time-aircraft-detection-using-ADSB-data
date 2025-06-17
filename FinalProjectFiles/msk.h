#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <sys/time.h>



void initChannel();

void demodMSK(uint8_t *rtlinbuff, int len);


#ifdef __cplusplus
}
#endif