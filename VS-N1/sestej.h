#include <stdint.h>
#include <stdio.h>

int32_t ImaNatankoTriDelitelje(int32_t n);

void PretvorbaUre(int16_t *str, int32_t t);

void IzpisBinarno(int32_t n);

int32_t SodoLih(int32_t n);

int32_t DeljivoS4(int32_t n);

int32_t StevecBitov(int16_t n);

int32_t BinarniPalindrom(int16_t n);

struct Opravilo{
    int16_t *ime;
    int8_t prioriteta;
    int8_t cas_izvajanja;
    struct Opravilo *naslednje;
};

int32_t *SpremeniBit(uint32_t *n, int32_t i);

