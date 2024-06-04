
//Divide 32 bit integer into half and then further half .Reverse the order of halves.

/*
 UPPER     LOWER   HALVES  FINAL RESULT
 0xDEAD   0XBEEF  ----->  0XEFBEADDE
 0XBE|0XEF|0XDE|0XAD---->SWAP UPPER AND LOWER HALF
 
 0XEF|OXBE|0XAD|0XDE  --->MIRROR ABOVE RESULT/REVERSE THE ORDER OF HALVES


*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// Function to swap upper and lower halves and then reverse the order of the bytes
uint32_t swapHalvesAndReverse(uint32_t org_num) {
    uint32_t upper_half = org_num >> 16; // Extract upper half (0xDEAD)
    uint32_t lower_half = org_num & 0xFFFF; // Extract lower half (0xBEEF)

    // Swap bytes within each half
    upper_half = (upper_half >> 8) | ((upper_half & 0xFF) << 8); // 0xDEAD -> 0xADDE
    lower_half = (lower_half >> 8) | ((lower_half & 0xFF) << 8); // 0xBEEF -> 0xEFBE

    // Combine the swapped halves in reversed order
    uint32_t converted_num = (lower_half << 16) | upper_half; // 0xEFBEADDE
    return converted_num;
}

// Function to convert a 32-bit integer from little-endian to big-endian (or vice versa)
uint32_t convertEndian(uint32_t orig_hex) {
    if (orig_hex == 0) {
        fprintf(stderr, "Warning: Zero value provided, no conversion needed.\n");
        return orig_hex;
    }
    
    uint32_t ret = 0;
    while (orig_hex != 0) {
        ret <<= 8;
        ret |= orig_hex & 0xFF;
        orig_hex >>= 8;
    }
    return ret;
}

// Another approach to reverse the bytes of a 32-bit integer
uint32_t reverseBytes(uint32_t num) {
    if (num == 0) {
        fprintf(stderr, "Warning: Zero value provided, no reversal needed.\n");
        return num;
    }

    unsigned char bytes[4];
    bytes[0] = num & 0xFF;
    bytes[1] = (num >> 8) & 0xFF;
    bytes[2] = (num >> 16) & 0xFF;
    bytes[3] = (num >> 24) & 0xFF;
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}

int main() {
    uint32_t x = 0xDEADBEEF;
    unsigned char *c = (unsigned char *)&x;

    if (c == NULL) {
        fprintf(stderr, "Error: Null pointer encountered for endian check.\n");
        return 1;
    }
    
    if (*c == 0xEF)
        printf("System is Little Endian\n");
    else
        printf("System is Big Endian\n");
    
    printf("Original value: 0x%X\n", x);
    
    // Convert little-endian to big-endian
    uint32_t bigEndian = convertEndian(x);
    if (bigEndian == x) {
        printf("No change in endian conversion: 0x%X\n", bigEndian);
    } else {
        printf("After conversion to big-endian: 0x%X\n", bigEndian);
    }
    
    // Divide the 32-bit integer into halves, swap them, and reverse the bytes
    uint32_t swappedAndReversed = swapHalvesAndReverse(x);
    if (swappedAndReversed == x) {
        printf("No change in swap and reverse: 0x%X\n", swappedAndReversed);
    } else {
        printf("After swapping halves and reversing: 0x%X\n", swappedAndReversed);
    }
    
    // Convert the integer into a 4-character array and swap the characters
    uint32_t reversedArray = reverseBytes(x);
    if (reversedArray == x) {
        printf("No change in byte reversal: 0x%X\n", reversedArray);
    } else {
        printf("After byte reversal: 0x%X\n", reversedArray);
    }
    
    return 0;
}
