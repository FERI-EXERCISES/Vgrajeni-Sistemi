//
// Created by simon on 03/10/2023.
//

#include "sestej.h"

int32_t ImaNatankoTriDelitelje(int32_t n) {
    int32_t stevec = 0;
    for (int32_t i = 1; i <= n; i++) {
        if (n % i == 0) {
            stevec++;
        }
    }
    return stevec == 3;
}

void PretvorbaUre(int16_t *str, int32_t t) {
    int32_t ure = t / 3600;
    int32_t minute = (t - ure * 3600) / 60;
    int32_t sekunde = t - ure * 3600 - minute * 60;
    int32_t milisekunde = t - ure * 3600 - minute * 60 - sekunde;

    sprintf((char *) str, "%02dh %02dm %02ds %03dms", ure, minute, sekunde, milisekunde);
}

void IzpisBinarno(int32_t n) {
    for (int32_t i = 31; i >= 0; i--) {
        printf("%d", (n >> i) & 1);
    }
    printf("\n");
}

int32_t SodoLih(int32_t n) {
    int32_t last_bit = n & 1;
    return last_bit == 0;
}

int32_t DeljivoS4(int32_t n) {
    int32_t last_2_bits = n & 3;

    return (last_2_bits | 0) == 0;
}

int32_t StevecBitov(int16_t n) {
    int32_t stevec = 0;
    for (int32_t i = 0; i < 16; i++) {
        if ((n >> i) & 1) {
            stevec++;
        }
    }
    return stevec;
}

int32_t BinarniPalindrom(int16_t n) {
    for (int i = 0; i < 8; ++i) {
        if (((n >> i) & 1) != (n >> (15 - i) & 1)) {
            return 0;
        }
    }
    return 1;
}

int32_t *SpremeniBit(uint32_t *n, int32_t i) {
    printf("Originalna vrednost: %d", n);
    *n |= (1u << i);
    printf("Spremenjena vrednost: %d", n);
    return n;
}



