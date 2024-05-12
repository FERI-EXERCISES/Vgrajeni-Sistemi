#include "sestej.h"

int main() {
    // 1. sklop
    // a. Delitelji
    for (int i = 1; i <= 500; ++i) {
        if (ImaNatankoTriDelitelje(i)) {
            printf("%d ", i);
        }
    }
    printf("\n");

    // b. Pretvorba ure
    int32_t t = 123456;
    int16_t str[16];
    PretvorbaUre(str, t);
    printf("%s\n", str);

    // 2. sklop
    // a. Izpis stevila kot binarno
    int32_t n = 123456;
    IzpisBinarno(n);

    // b. Sodo ali liho stevilo
    SodoLih(n) ? printf("Sodo\n") : printf("Liho\n");

    int32_t m = 123442;
    DeljivoS4(m) ? printf("Deljivo s 4\n") : printf("Ni deljivo s 4\n");

    // 3. sklop
    // a. Stevec bitov
    printf("Stevilo bitov: %d\n", StevecBitov(-7));

    // d. Binarni palindrom
    int16_t palindrom = -21931;
    BinarniPalindrom(palindrom) ? printf("Binarni palindrom\n") : printf("Ni binarni palindrom\n");

    // 4. sklop
    // a. Direkten dostop do pomnilnika
    uint32_t naslov = 0x40021400;
    int32_t bit = 5;
    //SpremeniBit(&naslov, bit);

    // b. Enosmerno povezan seszam opravil
    struct Opravilo opravilo1 = {(int16_t *) "opravilo1", 1, 1, NULL};
    struct Opravilo opravilo2 = {(int16_t *) "opravilo2", 2, 2, NULL};
    struct Opravilo opravilo3 = {(int16_t *) "opravilo3", 3, 3, NULL};

    opravilo1.naslednje = &opravilo2;
    opravilo2.naslednje = &opravilo3;

    return 0;
}